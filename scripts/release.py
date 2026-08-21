#!/usr/bin/env python3
'''
Create a github release for the AM32-bootloader repository.

The version comes from Inc/version.h, giving a tag of the form vNN.0.0. The
script checks the version has not already been released, then builds every
bootloader and bootloader updater in a temporary detached git worktree at the
commit being released, so that nothing uncommitted or untracked can end up in
the release. The updaters are packaged into the two zip files used by the
release, the tag is created and pushed, and a draft github release is created
with auto generated notes. The assets are uploaded and checked against the
local files, and only then is the release published.

If the script fails after the tag is pushed, re-run it with --resume. That
rebuilds the tagged commit from scratch and uploads whatever is missing.
'''

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

DEFAULT_REPO = "am32-firmware/AM32-bootloader"

# the two updater zips, by the file extension that goes in them
UPDATER_ZIPS = {
    "hex": "AM32-bootloader-updaters-hex.zip",
    "amj": "AM32-bootloader-updaters-amj.zip",
}


class Failure(Exception):
    pass


def info(msg):
    print("== %s" % msg, flush=True)


def warn(msg):
    print("WARNING: %s" % msg, flush=True)


def run(cmd, cwd=None, capture=True, check=True, quiet=False):
    '''run a command, returning the CompletedProcess'''
    if not quiet:
        info("run: %s" % " ".join(cmd))
    try:
        p = subprocess.run(cmd, cwd=cwd, check=False,
                           stdout=subprocess.PIPE if capture else None,
                           stderr=subprocess.PIPE if capture else None,
                           text=True)
    except FileNotFoundError:
        raise Failure("command not found: %s" % cmd[0])
    if check and p.returncode != 0:
        err = ((p.stdout or "") + (p.stderr or "")).strip() if capture else ""
        raise Failure("command failed (%d): %s%s" % (
            p.returncode, " ".join(cmd), "\n" + err if err else ""))
    return p


def out(cmd, cwd=None, check=True):
    '''run a command, returning stripped stdout'''
    return run(cmd, cwd=cwd, capture=True, check=check, quiet=True).stdout.strip()


def need_tool(name):
    if shutil.which(name) is None:
        raise Failure("%s is not installed or not in PATH" % name)


def repo_root(script_path):
    '''find and sanity check the repository root'''
    root = os.path.dirname(os.path.dirname(os.path.abspath(script_path)))
    for f in ["Makefile", "Inc/version.h", "bootloader/main.c"]:
        if not os.path.exists(os.path.join(root, f)):
            raise Failure("%s not found, %s is not an AM32-bootloader checkout" % (f, root))
    top = out(["git", "rev-parse", "--show-toplevel"], cwd=root)
    if os.path.realpath(top) != os.path.realpath(root):
        raise Failure("git toplevel %s does not match %s" % (top, root))
    return root


def get_version(dirname):
    '''get BOOTLOADER_VERSION from Inc/version.h'''
    path = os.path.join(dirname, "Inc/version.h")
    txt = open(path, "r").read()
    matches = re.findall(r'^\s*#define\s+BOOTLOADER_VERSION\s+(\d+)\s*$', txt, re.M)
    if len(matches) != 1:
        raise Failure("expected exactly one BOOTLOADER_VERSION in %s, found %u" % (path, len(matches)))
    return int(matches[0])


def parse_remote(url):
    '''return owner/repo for a github remote url'''
    m = re.match(r'^(?:git@[^:]+:|(?:https?|ssh)://[^/]+/)(.+?)(?:\.git)?/?$', url)
    if m is None:
        raise Failure("cannot parse github remote url '%s'" % url)
    return m.group(1)


def check_repo(root, expected):
    url = out(["git", "remote", "get-url", "origin"], cwd=root)
    got = parse_remote(url)
    if got.lower() != expected.lower():
        raise Failure("origin is %s, expected %s (use --repo to override)" % (got, expected))
    info("origin is %s" % got)


def check_gh_auth(repo):
    '''check gh is usable and we have write access, returning the default branch'''
    p = run(["gh", "auth", "status"], check=False, quiet=True)
    if p.returncode != 0:
        raise Failure("gh is not authenticated, run 'gh auth login'\n%s" % (p.stdout or p.stderr))
    txt = out(["gh", "repo", "view", repo, "--json", "defaultBranchRef,viewerPermission"])
    d = json.loads(txt)
    perm = d.get("viewerPermission")
    if perm not in ["ADMIN", "MAINTAIN", "WRITE"]:
        raise Failure("you have %s permission on %s, need write access to make a release" % (perm, repo))
    return d["defaultBranchRef"]["name"]


def fetch(root):
    run(["git", "fetch", "--tags", "origin"], cwd=root, capture=False)


def remote_tag_sha(root, tag):
    '''return the commit a tag points at on origin, or None if it does not exist'''
    txt = out(["git", "ls-remote", "--tags", "origin", "refs/tags/%s" % tag,
               "refs/tags/%s^{}" % tag], cwd=root)
    shas = {}
    for line in txt.splitlines():
        sha, ref = line.split("\t")
        shas[ref] = sha
    # the peeled ref is the commit for an annotated tag
    peeled = shas.get("refs/tags/%s^{}" % tag)
    if peeled is not None:
        return peeled
    return shas.get("refs/tags/%s" % tag)


def local_tag_sha(root, tag):
    '''return the commit a local tag points at, or "" if there is no such tag'''
    return out(["git", "rev-parse", "--verify", "--quiet", "refs/tags/%s^{commit}" % tag],
               cwd=root, check=False)


def all_remote_tags(root):
    tags = []
    for line in out(["git", "ls-remote", "--tags", "origin"], cwd=root).splitlines():
        ref = line.split("\t")[-1]
        if ref.startswith("refs/tags/") and not ref.endswith("^{}"):
            tags.append(ref[len("refs/tags/"):])
    return tags


def get_release(repo, tag, fields="isDraft,url,apiUrl,assets"):
    '''return the release for a tag, or None if there is no such release'''
    p = run(["gh", "release", "view", tag, "--repo", repo, "--json", fields],
            check=False, quiet=True)
    if p.returncode == 0:
        return json.loads(p.stdout)
    err = ((p.stdout or "") + (p.stderr or "")).lower()
    if "release not found" in err or "404" in err:
        return None
    # do not treat an API error as "no release exists"
    raise Failure("could not check for an existing release of %s:\n%s" % (
        tag, (p.stderr or "").strip()))


def check_not_released(root, repo, tag, version, allow_same_major):
    '''check the tag and the version have not been released'''
    if local_tag_sha(root, tag) != "":
        raise Failure("tag %s already exists locally (use --resume to continue a failed release)" % tag)
    if remote_tag_sha(root, tag) is not None:
        raise Failure("tag %s already exists on origin (use --resume to continue a failed release)" % tag)
    if get_release(repo, tag) is not None:
        raise Failure("a github release for %s already exists (use --resume to continue a failed release)" % tag)

    same_major = sorted([t for t in all_remote_tags(root)
                         if re.match(r'^v%u\.\d+\.\d+$' % version, t)])
    if same_major:
        msg = "version %u is already released as %s" % (version, ", ".join(same_major))
        if not allow_same_major:
            raise Failure("%s\nbump BOOTLOADER_VERSION in Inc/version.h, or use --tag with "
                          "--allow-same-major for a patch release" % msg)
        warn(msg)
    info("%s is not released yet" % tag)


def check_tree(root, allow_dirty):
    '''check there are no modified tracked files'''
    dirty = out(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root)
    if dirty != "":
        if not allow_dirty:
            raise Failure("working tree has modified tracked files:\n%s\n"
                          "commit or stash them (or use --allow-dirty)" % dirty)
        warn("working tree has modified tracked files, they are NOT in the release:\n%s" % dirty)


def check_is_tip(root, sha, default_branch, allow_unpushed):
    '''check the commit being released is the tip of the default branch on origin'''
    remote_ref = "origin/%s" % default_branch
    tip = out(["git", "rev-parse", "--verify", "--quiet", "%s^{commit}" % remote_ref],
              cwd=root, check=False)
    if tip == "":
        raise Failure("%s not found, check the origin remote" % remote_ref)
    if tip == sha:
        info("%s is the tip of %s" % (sha[:10], remote_ref))
        return
    p = run(["git", "merge-base", "--is-ancestor", sha, remote_ref], cwd=root,
            check=False, quiet=True)
    if p.returncode == 0:
        msg = "%s is behind the tip of %s (%s)" % (sha[:10], remote_ref, tip[:10])
    else:
        msg = "%s is not on %s, push your commits first" % (sha[:10], remote_ref)
    if not allow_unpushed:
        raise Failure("%s (or use --allow-unpushed)" % msg)
    warn(msg)


def link_tools(root, wtdir):
    '''link the toolchains from the main checkout into the build worktree'''
    src = os.path.join(root, "tools")
    dst = os.path.join(wtdir, "tools")
    if not os.path.isdir(src):
        # no toolchain yet, do_build() installs it and links again
        return
    if not os.path.isdir(dst):
        os.mkdir(dst)
    for name in sorted(os.listdir(src)):
        target = os.path.join(dst, name)
        if os.path.exists(target) or os.path.islink(target):
            continue
        os.symlink(os.path.realpath(os.path.join(src, name)), target)


def setup_build_dir(root, tag, sha):
    '''create a fresh detached worktree at sha to build the release in'''
    wtdir = os.path.join(tempfile.gettempdir(), "am32-release-%s" % tag)

    worktrees = []
    for line in out(["git", "worktree", "list", "--porcelain"], cwd=root).splitlines():
        if line.startswith("worktree "):
            worktrees.append(os.path.realpath(line[len("worktree "):]))

    if os.path.exists(wtdir):
        if os.path.realpath(wtdir) not in worktrees:
            raise Failure("%s exists but is not a git worktree, remove it and re-run" % wtdir)
        info("removing old build directory %s" % wtdir)
        run(["git", "worktree", "remove", "--force", wtdir], cwd=root)
    if os.path.exists(wtdir):
        raise Failure("could not remove %s" % wtdir)

    info("creating build worktree %s at %s" % (wtdir, sha[:10]))
    run(["git", "worktree", "add", "--detach", wtdir, sha], cwd=root, capture=False)
    check_build_dir(root, wtdir, sha)
    link_tools(root, wtdir)
    return wtdir


def check_build_dir(root, wtdir, sha):
    '''check the build worktree is still a pristine checkout of sha'''
    if out(["git", "rev-parse", "HEAD^{commit}"], cwd=wtdir) != sha:
        raise Failure("build worktree %s is no longer at %s" % (wtdir, sha[:10]))
    # obj/ and tools/ are in .gitignore, so a clean checkout stays clean when built
    dirty = out(["git", "status", "--porcelain"], cwd=wtdir)
    if dirty != "":
        raise Failure("build worktree %s is not clean:\n%s" % (wtdir, dirty))


def remove_build_dir(root, wtdir):
    run(["git", "worktree", "remove", "--force", wtdir], cwd=root, check=False)
    if os.path.exists(wtdir):
        warn("could not remove build directory %s" % wtdir)


def make_query(builddir, expr):
    '''ask make to expand an expression'''
    p = run(["make", "--eval=__query__: ; @echo %s" % expr, "__query__"],
            cwd=builddir, check=False, quiet=True)
    if p.returncode != 0:
        raise Failure("make query '%s' failed:\n%s" % (expr, (p.stderr or "").strip()))
    lines = [l.strip() for l in p.stdout.splitlines() if l.strip() != ""]
    if len(lines) != 1:
        raise Failure("unexpected output from make query '%s':\n%s" % (expr, p.stdout))
    return lines[0].split()


def get_build_lists(builddir):
    '''get the bootloader targets, updater targets and native (no hex) MCUs'''
    builds = make_query(builddir, "$(ALL_BUILDS)")
    updaters = make_query(builddir, "$(BLU_BUILDS)")
    native = make_query(builddir, "$(foreach M,$(MCU_TYPES),$(if $(NATIVE_$(M)),$(M)))")
    if not builds:
        raise Failure("make returned no bootloader targets")
    if not updaters:
        raise Failure("make returned no bootloader updater targets")
    for t in builds + updaters:
        if not re.match(r'^AM32_[A-Z0-9]+_(BOOTLOADER|BL_UPDATER)_', t):
            raise Failure("unexpected make target name '%s'" % t)
    for lst, what in [(builds, "bootloader"), (updaters, "updater")]:
        dups = sorted(set([t for t in lst if lst.count(t) > 1]))
        if dups:
            raise Failure("duplicate %s targets: %s" % (what, ", ".join(dups)))
    return builds, updaters, native


def target_mcu(target):
    '''the MCU of a target, eg AM32_L431_BOOTLOADER_PB4_CAN -> L431'''
    return target.split("_")[1]


def do_build(root, builddir, jobs, install_tools):
    '''build all bootloaders and updaters'''
    p = run(["make", "check_tools"], cwd=builddir, check=False, quiet=True)
    if p.returncode != 0:
        if not install_tools:
            raise Failure("the toolchain is not installed, run 'make arm_sdk_install' "
                          "(or use --install-tools)")
        info("installing toolchain")
        run(["make", "arm_sdk_install"], cwd=root, capture=False)
        link_tools(root, builddir)
        run(["make", "check_tools"], cwd=builddir, capture=False)

    run(["make", "-j%u" % jobs], cwd=builddir, capture=False)
    run(["make", "-j%u" % jobs, "updaters"], cwd=builddir, capture=False)


def check_hex(path):
    '''basic sanity check of an intel hex file'''
    if not os.path.exists(path):
        raise Failure("missing build output %s" % path)
    data = open(path, "r", errors="replace").read()
    if len(data) < 100:
        raise Failure("%s is too small (%u bytes)" % (path, len(data)))
    if not data.startswith(":"):
        raise Failure("%s is not an intel hex file" % path)
    if ":00000001FF" not in data:
        raise Failure("%s has no end of file record" % path)


def check_artifacts(builddir, version, builds, updaters, native, sha):
    '''check every expected build output is present and sane'''
    objdir = os.path.join(builddir, "obj")
    suffix = "_V%u" % version

    hexes = []
    for t in builds:
        base = os.path.join(objdir, t + suffix)
        if target_mcu(t) in native:
            # native (SITL) builds produce no hex and are not released
            if not os.path.exists(base + ".elf"):
                raise Failure("missing build output %s.elf" % base)
            continue
        check_hex(base + ".hex")
        hexes.append(base + ".hex")

    # the build must not produce any other bootloader hex
    found = set()
    for f in os.listdir(objdir):
        if re.match(r'^AM32_.*_BOOTLOADER_.*\.hex$', f):
            found.add(os.path.join(objdir, f))
    extra = found - set(hexes)
    if extra:
        raise Failure("unexpected bootloader hex files in obj:\n%s" % "\n".join(sorted(extra)))

    updater_files = {"hex": [], "amj": []}
    for t in updaters:
        base = os.path.join(objdir, t + suffix)
        check_hex(base + ".hex")
        updater_files["hex"].append(base + ".hex")
        amj = base + ".amj"
        if not os.path.exists(amj):
            raise Failure("missing build output %s" % amj)
        try:
            d = json.load(open(amj, "r"))
        except Exception as ex:
            raise Failure("%s is not valid json: %s" % (amj, ex))
        for k in ["type", "mcuType", "pin", "githash", "version", "hex"]:
            if k not in d:
                raise Failure("%s has no '%s' key" % (amj, k))
        if d["githash"] != sha:
            raise Failure("%s was built from %s, expected %s" % (amj, d["githash"][:10], sha[:10]))
        if d["version"] != "V%u" % version:
            raise Failure("%s has version %s, expected V%u" % (amj, d["version"], version))
        updater_files["amj"].append(amj)

    info("built %u bootloader hex files and %u updaters" % (len(hexes), len(updaters)))
    return sorted(hexes), updater_files


def make_zips(builddir, updater_files):
    '''create the updater zip files, matching the layout used by CI'''
    objdir = os.path.join(builddir, "obj")
    zips = []
    for ext, name in sorted(UPDATER_ZIPS.items()):
        path = os.path.join(objdir, name)
        files = sorted(updater_files[ext])
        if not files:
            raise Failure("no .%s updater files to put in %s" % (ext, name))
        if os.path.exists(path):
            os.unlink(path)
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
            for f in files:
                zf.write(f, arcname=os.path.basename(f))
        # verify what we just wrote
        with zipfile.ZipFile(path, "r") as zf:
            bad = zf.testzip()
            if bad is not None:
                raise Failure("%s is corrupt (%s)" % (path, bad))
            if sorted(zf.namelist()) != sorted([os.path.basename(f) for f in files]):
                raise Failure("%s does not contain the expected files" % path)
        info("created %s with %u files (%u bytes)" % (name, len(files), os.path.getsize(path)))
        zips.append(path)
    return zips


def confirm(prompt, assume_yes):
    if assume_yes:
        return
    if not sys.stdin.isatty():
        raise Failure("stdin is not a tty, use --yes to run non-interactively")
    if input("%s [y/N] " % prompt).strip().lower() not in ["y", "yes"]:
        raise Failure("aborted by user")


def create_tag(root, tag, sha):
    '''create and push the release tag, tolerating a re-run'''
    rsha = remote_tag_sha(root, tag)
    if rsha == sha:
        info("tag %s is already on origin" % tag)
        run(["git", "fetch", "--tags", "origin"], cwd=root, check=False, quiet=True)
        return
    if rsha is not None:
        raise Failure("tag %s on origin points at %s, not %s" % (tag, rsha[:10], sha[:10]))

    local = local_tag_sha(root, tag)
    if local == "":
        run(["git", "tag", "-a", tag, "-m", tag, sha], cwd=root)
    elif local != sha:
        raise Failure("local tag %s points at %s, not %s" % (tag, local[:10], sha[:10]))

    p = run(["git", "push", "origin", "refs/tags/%s" % tag], cwd=root, capture=False, check=False)
    if p.returncode != 0:
        # the push may still have been accepted, check before undoing anything
        rsha = remote_tag_sha(root, tag)
        if rsha == sha:
            warn("git push reported an error but tag %s is on origin, carrying on" % tag)
            return
        if rsha is not None:
            raise Failure("tag %s on origin points at %s, not %s" % (tag, rsha[:10], sha[:10]))
        run(["git", "tag", "-d", tag], cwd=root, check=False)
        raise Failure("failed to push tag %s to origin" % tag)
    info("pushed tag %s" % tag)


def release_assets(repo, tag):
    '''get the assets of a release, including their sha256 digests'''
    rel = get_release(repo, tag, fields="apiUrl,isDraft")
    if rel is None:
        raise Failure("release %s not found" % tag)
    m = re.match(r'^.*/releases/(\d+)$', rel["apiUrl"])
    if m is None:
        raise Failure("cannot get the release id from %s" % rel["apiUrl"])
    txt = out(["gh", "api", "repos/%s/releases/%s/assets?per_page=100" % (repo, m.group(1)),
               "--paginate"])
    return rel, json.loads(txt)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def upload_assets(repo, tag, assets, batch=20):
    '''upload the release assets, only adding missing ones to a published release'''
    rel, got = release_assets(repo, tag)
    have = set([a["name"] for a in got])
    todo = assets
    clobber = ["--clobber"]
    if not rel["isDraft"]:
        # never overwrite an asset of a release that is already public
        todo = [a for a in assets if os.path.basename(a) not in have]
        clobber = []
        warn("release %s is already published, uploading %u missing assets only" % (tag, len(todo)))
    if not todo:
        info("all assets are already uploaded")
        return
    n = 0
    for i in range(0, len(todo), batch):
        chunk = todo[i:i+batch]
        n += len(chunk)
        info("uploading assets %u/%u" % (n, len(todo)))
        cmd = ["gh", "release", "upload", tag, "--repo", repo] + clobber + chunk
        p = run(cmd, check=False, capture=False, quiet=True)
        if p.returncode != 0:
            # the retry may need to replace a partial upload, but only of the
            # assets in this chunk, which we uploaded ourselves
            warn("upload failed, retrying")
            run(cmd + ([] if clobber else ["--clobber"]), capture=False, quiet=True)


def verify_assets(repo, tag, assets):
    '''check the release has exactly the expected assets, with the right contents'''
    rel, got = release_assets(repo, tag)
    local = dict([(os.path.basename(a), a) for a in assets])
    names = set([a["name"] for a in got])

    missing = set(local.keys()) - names
    if missing:
        raise Failure("assets missing from the release:\n%s" % "\n".join(sorted(missing)))
    extra = names - set(local.keys())
    if extra:
        raise Failure("release has unexpected assets:\n%s" % "\n".join(sorted(extra)))
    ndigest = 0
    for a in got:
        path = local[a["name"]]
        if a["state"] != "uploaded":
            raise Failure("asset %s is in state '%s'" % (a["name"], a["state"]))
        if a["size"] != os.path.getsize(path):
            raise Failure("asset %s is %u bytes on github, expected %u" % (
                a["name"], a["size"], os.path.getsize(path)))
        digest = a.get("digest")
        if digest is not None:
            if digest != "sha256:%s" % sha256(path):
                raise Failure("asset %s does not match the local file" % a["name"])
            ndigest += 1
    info("all %u assets uploaded, %u checked by sha256" % (len(local), ndigest))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default=DEFAULT_REPO, help="github repository")
    parser.add_argument("--tag", default=None, help="release tag (default v<VERSION>.0.0)")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 4, help="parallel make jobs")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    parser.add_argument("--dry-run", action="store_true",
                        help="do all the checks and the build, but do not tag or release")
    parser.add_argument("--draft", action="store_true", help="leave the release as a draft")
    parser.add_argument("--resume", action="store_true",
                        help="continue a release that failed after the tag was pushed")
    parser.add_argument("--keep-build", action="store_true",
                        help="keep the build worktree when finished")
    parser.add_argument("--install-tools", action="store_true",
                        help="run 'make arm_sdk_install' if the toolchain is missing")
    parser.add_argument("--allow-dirty", action="store_true",
                        help="allow modified tracked files in the working tree")
    parser.add_argument("--allow-unpushed", action="store_true",
                        help="allow HEAD to not be the tip of the default branch on origin")
    parser.add_argument("--allow-same-major", action="store_true",
                        help="allow a release when this version is already released")
    args = parser.parse_args()

    if args.jobs < 1:
        raise Failure("--jobs must be at least 1")

    for tool in ["git", "gh", "make", "python3"]:
        need_tool(tool)

    root = repo_root(__file__)
    info("repository %s" % root)

    version = get_version(root)
    tag = args.tag if args.tag is not None else "v%u.0.0" % version
    m = re.match(r'^v(\d+)\.\d+\.\d+$', tag)
    if m is None:
        raise Failure("tag '%s' is not of the form vX.Y.Z" % tag)
    # the release version comes from the tag, and is checked against the
    # version.h of the commit being released further down
    version = int(m.group(1))

    default_branch = check_gh_auth(args.repo)
    check_repo(root, args.repo)
    fetch(root)

    if args.resume:
        # a resumed release is defined by the tag that was already pushed, not
        # by whatever HEAD happens to be now
        sha = remote_tag_sha(root, tag)
        if sha is None:
            raise Failure("tag %s is not on origin, there is nothing to resume" % tag)
        if run(["git", "cat-file", "-e", "%s^{commit}" % sha], cwd=root,
               check=False, quiet=True).returncode != 0:
            raise Failure("commit %s of tag %s is not in this repository" % (sha[:10], tag))
        local = local_tag_sha(root, tag)
        if local not in ["", sha]:
            raise Failure("local tag %s points at %s, not %s" % (tag, local[:10], sha[:10]))
        rel = get_release(args.repo, tag)
        info("resuming release of %s at %s (release %s)" % (
            tag, sha[:10], "exists" if rel is not None else "not created yet"))
    else:
        version_head = get_version(root)
        if version != version_head:
            raise Failure("tag %s does not match BOOTLOADER_VERSION %u in Inc/version.h" % (
                tag, version_head))
        check_tree(root, args.allow_dirty)
        sha = out(["git", "rev-parse", "HEAD^{commit}"], cwd=root)
        check_is_tip(root, sha, default_branch, args.allow_unpushed)
        check_not_released(root, args.repo, tag, version, args.allow_same_major)

    info("releasing %s from commit %s" % (tag, sha))

    builddir = setup_build_dir(root, tag, sha)
    ok = False
    try:
        # the version must come from the commit being released, not from an
        # uncommitted edit of version.h
        build_version = get_version(builddir)
        if build_version != version:
            raise Failure("commit %s has BOOTLOADER_VERSION %u, not %u as in tag %s, "
                          "commit your version bump" % (sha[:10], build_version, version, tag))

        builds, updaters, native = get_build_lists(builddir)
        do_build(root, builddir, args.jobs, args.install_tools)
        check_build_dir(root, builddir, sha)
        hexes, updater_files = check_artifacts(builddir, version, builds, updaters, native, sha)
        zips = make_zips(builddir, updater_files)
        assets = zips + hexes

        print("")
        info("ready to release %s" % tag)
        info("  repository: %s" % args.repo)
        info("  commit:     %s" % sha)
        info("              %s" % out(["git", "log", "-1", "--format=%s", sha], cwd=root))
        info("  assets:     %u (%u bootloader hex files, %u zips)" % (
            len(assets), len(hexes), len(zips)))
        print("")

        if args.dry_run:
            info("dry run, not tagging or creating the release")
            info("build outputs are in %s/obj" % builddir)
            args.keep_build = True
            ok = True
            return

        confirm("tag %s, push it to origin and create the github release?" % tag, args.yes)

        # re-check that nothing moved under us during the build
        fetch(root)
        if not args.resume:
            check_tree(root, args.allow_dirty)
            if out(["git", "rev-parse", "HEAD^{commit}"], cwd=root) != sha:
                raise Failure("HEAD changed during the build, re-run the release")
            check_is_tip(root, sha, default_branch, args.allow_unpushed)

        create_tag(root, tag, sha)

        if get_release(args.repo, tag) is None:
            try:
                run(["gh", "release", "create", tag, "--repo", args.repo, "--title", tag,
                     "--generate-notes", "--verify-tag", "--draft"], capture=False)
            except Failure:
                print("\nThe tag was pushed but the release was not created. Re-run with --resume,\n"
                      "or remove the tag with:\n"
                      "  git push origin :refs/tags/%s\n  git tag -d %s" % (tag, tag),
                      file=sys.stderr)
                raise
        else:
            info("using the existing release for %s" % tag)

        upload_assets(args.repo, tag, assets)
        verify_assets(args.repo, tag, assets)

        d = get_release(args.repo, tag, fields="isDraft,url")
        if args.draft:
            info("release left as a draft")
        elif d["isDraft"]:
            run(["gh", "release", "edit", tag, "--repo", args.repo, "--draft=false", "--latest"],
                capture=False)
            # publishing must not have changed the assets
            verify_assets(args.repo, tag, assets)

        d = get_release(args.repo, tag, fields="isDraft,url")
        if d["isDraft"] and not args.draft:
            raise Failure("release %s is still a draft" % tag)
        info("release %s %s: %s" % (tag, "drafted" if d["isDraft"] else "published", d["url"]))
        ok = True
    finally:
        if ok and not args.keep_build:
            remove_build_dir(root, builddir)
        elif not ok:
            warn("build directory kept for inspection: %s" % builddir)


if __name__ == '__main__':
    try:
        main()
    except Failure as ex:
        print("\nERROR: %s" % ex, file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(1)
