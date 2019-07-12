Notes on how to configure a raspberry pi as a data recorder companion computer for the version of the system developed in spring/summer 2019.

# Setup Rasbpian

1. Download Raspbian Buster Lite from one of these sources: (tested with the 2019-06-20 version)
    - https://www.raspberrypi.org/downloads/raspbian/
    - \\olympus\Projects\6614-Wake-Swarm-II\Technical Work\SoftwareandDrivers\Raspberry Pi data recorder setup prerequisites
2. Write this image to an SD card.  Last time we used:
    - SD Card: 64GB SanDisk *Extreme PRO*
    - Windows SD card writer: Balena Etcher
    - SD card drive: plugable (sic) SuperSpeed USB3
3. Find the volume named "boot" in "my computer".
4. Create an empty file at the root of the boot volume named "ssh" with no extension.  This tells the Pi to be reachable via SSH on startup.

# Setup on Pi

1. SSH into the Pi
    - Default username is "pi" and password is "raspberry"
    - To find it on the network, see https://code.crearecomputing.com/internal/training/blob/master/networking/tips-and-tricks.md
2. Run `sudo raspi-config` and enable SPI and UART.  Disable UART console access.
2. Make sure local time is correct with the date command.
6. Upgrade: `sudo apt update; sudo apt -y upgrade`
4. Install required libraries and tools: `sudo apt install -y git nano libboost-all-dev screen`
5. Install useful tools: `sudo apt install -y tree python netcat socat python-pip; sudo pip install spidev`
7. Cleanup: `sudo apt -y autoremove`

## Spinnaker setup

### Increase `usbfs` size
(Courtesy https://www.flir.com/support-center/iis/machine-vision/application-note/understanding-usbfs-on-linux/ )
1. Run `sudo nano /boot/cmdline.txt`
2. Add the following to the end of the line: ` usbcore.usbfs_memory_mb=1000`
3. Reboot

### Install

1. Copy spinnaker-1.23.0.27-armhf-Ubuntu16.04-pkg.tar.gz to `/home/creare` on the Pi.
2. Run `cd`
3. Run `tar -xzvf spinnaker-1.23.0.27-armhf-Ubuntu16.04-pkg.tar.gz`
4. Run `cd spinnaker-1.23.0.27-armhf/`
5. Run `printf "y\ny\ncreare\ny\ny\ny\nn\n" | ./install_spinnaker_arm.sh`
6. Run `sudo apt -y install libraw1394-11 libusb-1.0-0`

## OpenCV setup
(Some steps courtesy https://www.pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/)
1. Run `sudo apt update && sudo apt -y upgrade`
1. Run `sudo apt -y install build-essential cmake unzip pkg-config`
1. Run `sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
 `sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libcanberra-gtk* libatlas-base-dev gfortran python3-dev`
sudo pip3 install numpy
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.0.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.0.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.0.0 opencv
mv opencv_contrib-4.0.0 opencv_contrib
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=ON \
    -D BUILD_TESTS=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF ..

sudo nano /etc/dphys-swapfile
Replace CONF_SWAPSIZE=100 with CONF_SWAPSIZE=2048
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
    
    
## Creare software setup
Mount sd card
add to fstab
`git clone`
`git submodule update`
Automatic startup script when you write that