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

#### As of Jetpack 4
1. ????

#### Up to Jetpack 3
1. Run `sudo nano /etc/default/grub`
2. Find `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
3. Replace with `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"`
4. Exit with `^x <enter> Y`
4. Run `sudo update-grub`

### Install

1. Copy spinnaker-1.23.0.27-arm64-Ubuntu18.04-pkg.tar.gz to `/home/creare` on the Jetson.
2. Run `cd`
3. Run `tar -xzvf spinnaker-1.23.0.27-arm64-Ubuntu18.04-pkg.tar.gz`
4. Run `cd spinnaker-1.23.0.27-arm64/`
5. Run `printf "y\ny\ncreare\ny\ny\ny\nn\n" | ./install_spinnaker_arm.sh`
6. Make sure it worked by running `SpinView_QT` (if you have X windows forwarding working)

## Creare software setup
Mount sd card
add to fstab
`git clone`
`git submodule update`
Automatic startup script when you write that