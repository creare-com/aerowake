# Steps to set up pi for the Reel

- Create base 30GB image
- Enable SPI, I2C, Serial
- Reboot
- Set time
- Apt update/upgrade
- `sudo apt -y install git python-pip i2c-tools ipython libboost-all-dev libopencv-dev python3 python3-pip ipython3`
    - Opencv is a big dependency
    - But it means we can just use the FlightRecorder without modification to make recordings of the GCS position
- `sudo pip3 install cython Adafruit-ADS1x15`
- `git clone --recursive https://gitlab.com/creare-com/wake-swarm/aerowake.git`
    - Recursive option is required to get the git submodule for mavlink support in the FlightRecorder
- `cd ~/aerowake/Flight\ source/gcs/reel/ReelController/lib`
- `bash install.sh`
- `cd ~/aerowake/Flight\ source/gcs/reel/ReelController`
- `bash make`

# Build FlightRecorder
- Download the "armhf" build of the Spinnaker API from FLIR, under the Ubuntu section (spinnaker-2.0.0.147-armhf-pkg.tar.gz)
    - Justification: Spinnaker is used by the FlightRecorder to record images from FLIR cameras.
    - The GCS doesn't need that at all
    - We have two options:
        1. Install Spinnaker anyway and just don't record from any cameras
        2. Add preprocessor directives to the FlightRecorder so you can build it without camera support
    - I chose option 1 because it seemed easier.  We should switch to option 2 if at any time that makes more sense.
- Upload to `/home/pi`
- `tar -xzvf spinnaker-2.0.0.147-armhf-pkg.tar.gz`
- `cd spinnaker-2.0.0.147-armhf-pkg.tar.gz`
- `./install_spinnaker_arm.sh` 
    - Yes to all license terms
    - Yes add udev entry 
    - Indicate user `pi` to add to `flirimaging` group
    - Yes to expanding `usbfs`
- Reboot to make sure everything is good
- `cd ~/aerowake/Flight\ source/FlightRecorder`
- `make`

