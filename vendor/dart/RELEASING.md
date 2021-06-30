# Releasing DART from Open Robotics fork

Ubuntu Launchpad PPA (Personal Package Archive) system is being used to release
the Open Robotics fork of DART. The debian metadata is being hosted the same
development branch `release-6.10`.

The procedure consists in two steps:

  1. Produce a local tarball of the sources and upload the tarball to Ubuntu's
     servers to built binaries
  1. Imported binaries from Ubuntu PPA into packages.osrfoundation.org

## Prerequisites

### Credentials

  * For being able to upload to Open Robotics Ubuntu PPA:
     * Account in [Ubuntu One](https://login.ubuntu.com/)
     * Access to https://launchpad.net/~openrobotics
       * New members can apply after login in launchpad in that same page

### Step 0: System Configurations

  * Enviroment variables to use as gbp configurations for changelogs
    (optionally in .bashrc):
    ```
    export DEBEMAIL="user@openrobotics.org
    export DEBFULLNAME="Your name"
    ```

  * Software installation: [debian git-buildpackage
    tool](https://wiki.debian.org/PackagingWithGit) and dput (to upload
    packages to PPA)
    ```
    apt-get install -y git-buildpackage dput
    ```

## Step 1: Releasing into PPA

The first step will create a source package from DART git checkout in the local
system and upload it to Open Robotics PPA.

**One Ubuntu distribution needs to be released at a time**. The information
about target distribution (start with bionic) goes only in the Changelog entry. To
release multiple distribution, repeat this step 1 changing the changelog entry 
(focal is also supported) and uploading the new source package to the PPA.

A copy of [this fork](https://github.com/ignition-forks/dart) is required to be in the system.

```
cd dart
git checkout release-6.10
```

### Update changelog

 1. ```
    DISTRO=<UBUNTU_DISTRO> gbp dch --ignore-branch --no-git-author -D $DISTRO --force-distribution --new-version=6.10.0~osrf6~$(date +%Y%m%d)~$DISTRO~$(git rev-parse HEAD) --commit-msg 'New OSRF testing release' --commit
    ```
    (change UBUNTU_DISTRO by the target distribution name, i.e: focal. Check changelog by running `git diff HEAD~1`)

### Releasing in Ubuntu PPA

After updating the changelog, the directory is ready to generate the source package.

#### Generate source package

 1. Build source package files using gbp
    ```
    gbp buildpackage -S
    ```

#### Upload source package to Ubuntu's PPA

 1. Upload source package (source package files are located in the parent directory `..`)
    ```
    dput ppa:openrobotics/dart ../dart6_*_source.changes
    ```

 1. If everything is correct, update the repository
    ```
    git push origin release-6.10
    ```

The last command will upload the source package to openrobotics PPA and will create the binaries
for the selected Ubuntu distribution in all arches (supported architectures can be configured in PPA)

## Step 2: Copy packages to packages.osrfoundation.org

TODO
