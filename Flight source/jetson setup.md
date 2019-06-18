# Setup on configurator laptop

1. Obtain laptop with Ubuntu 18.04
2. Copy contents of `\\olympus\Projects\6614-Wake-Swarm-II\Technical Work\Software\2019-6-18 Jetson setup prerequisites` to ~/Downloads
3. Run:
````bash
cd ~/Downloads
sudo dpkg -i sdkmanager_0.9.12-4180_amd64.deb
sudo apt --fix-broken install
sudo dpkg -i sdkmanager_0.9.12-4180_amd64.deb
unzip J120_4.2.zip
cd J120_4.2
tar -xjvf J120_kernel.tar.bz2
````

4. Set the date on the laptop.  Later steps will fail mysteriously if you don't.  Command is: `sudo date MMDDHHmmYYYY`.
5. Also install python, to avert another mysterious failure: `sudo apt install python`
6. Graphically open NVIDIA SDK manager.  Log in, or you can try using offline mode with the files from Olympus.
7. Set to TX2, with no host support.
8. Accept license and click "next".  Supply sudo password.  Wait until you're asked "automatic setup" or "manual setup".  Click "skip", "skip", "finish and exit".
9. You should now have a folder at:
`~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra/`
    - (If you don't, try an automatic download rather than an offline installation.  Or, remove and reinstall the SDK manager, including the ~/.nvsdkm directory)
10. Run:
````bash
cp -r ~/Downloads/J120_4.2/J120_kernel/* ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra/
cd ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra/
sudo ./apply_binaries.sh
````

11. Restart the SDK manager and repeat the above steps, but don't skip the flash step.
    - Note: I've only had success with the "manual" option, not the automatic one.
    - They say to hold the "force recovery button" for 2 seconds.  There's a slight increase in power draw at just about exactly 2 seconds, which I suspect is the point where you need to be holding the button past.  So I hold the button for a full "1 mississippi, 2 mississippi, 3 mississippi" count every time, making sure I release the button only after the current draw steps up.

12. When asked to set up the Jetson, use the HDMI mini port and a USB keyboard.  Use the settings:
    - hostname: wakeswarm-jetson
    - username: creare
    - password: creare

# Setup on Jetson

1. SSH into the Jetson.
2. Install required libraries and tools: `sudo apt install -y git libboost-all-dev screen`
3. Install useful tools: `sudo apt install -y tree python nc socat python-pip`

## Spinnaker setup

### Increase `usbfs` size
(Courtesy https://www.flir.com/support-center/iis/machine-vision/application-note/understanding-usbfs-on-linux/)

1. Run `sudo nano /etc/default/grub`
2. Find `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
3. Replace with `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"`
4. Exit with `^x <enter> Y`
4. Run `sudo update-grub`

### Install

1. Copy spinnaker-1.23.0.27-arm64-Ubuntu18.04-pkg.tar.gz to `~/Downloads/` on the Jetson.
2. Run `cd ~/Downloads`
3. Run `tar -xzvf spinnaker-1.23.0.27-arm64-Ubuntu18.04-pkg.tar.gz`
4. Run `cd `
