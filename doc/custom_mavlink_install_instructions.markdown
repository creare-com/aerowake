# Installing custom MAVLINK message to be used by the GCS and Raspberry pi
These custom messages are specified at the bottom of the: third_party/mavlink/pymavlink/dialects/v10/gcs_pixhawk.xml file

## Removing existing pymavlink installations
From commandline issue
```bash
sudo pip uninstall pymavlink
``` 

## Installing pymavlink from source
From commandline issue the following commands:
```bash
cd ~
cd aerowake-mit/third_party/mavlink/pymavlink
sudo python setup.py install
```

## Editing the messages
Simply edit the file mentioned above with new message or parameters, and then re-run `setup.py install`

## Notes
* This custom version of pymavlink has to be installed on both the GCS and the onboard PI to work
   * Hence why I included it in the repository
   * I also kept the .git folder so that we could potentially pull upstream changes (bugfixes) from the pymavlink project
* If you don't want to wait for all the mavlink protocols to be built use: 
```bash
sudo su
export NOGEN=0
setup.py install
exit
```

