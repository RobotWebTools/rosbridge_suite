# Distributing rosbridge_suite (ER modifications)

The changes are only in rosbridge_library.
- we package only rosbrdige_library
- install rosbrdige_suite as usual
- update rosbrdige_library with ER modifications

## Packaging

### .deb

1. Prerequisities

```bash
sudo apt-get install python-bloom fakeroot
```

2. Bump version in `package.xml`

3. Create .deb package

```bash
cd rosbridge_library
bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro melodic
fakeroot debian/rules binary
```

4. Install/remove/switch version .deb package

```bash
# install (here 0.11.130 version)
sudo apt install ./ros-melodic-rosbridge-library_0.11.130-0bionic_amd64.deb

# remove
# sudo apt remove ros-melodic-rosbridge-library

# switch version (e.g. to original)
## list candidates
# apt-cache policy ros-melodic-rosbridge-library
## switch back to original (here to 0.11.13-1bionic.20210921.211116)
# apt-get install ros-melodic-rosbridge-library=0.11.13-1bionic.20210921.211116
```

## Future updates

In case of future updates of rosbridge-suite
- on version changes (0.12.x and later) we should apply our changes on top and repackage
- in case of minor changes (0.11.14 and later) we should consider updating
  - our update will not have potential bugfixes

If updating:
- `package.xml` version should be appended with extra "0"
  -  e.g. 0.11.13 -> 0.11.130
- `package.xml` description should have information about our modifications
- `CHANGELOG.rst` should be updated with our change

