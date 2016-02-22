#! /bin/sh
### BEGIN INIT INFO
# Provides:          <your script name>
# Required-Start:    $all
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Manage my cool stuff
### END INIT INFO

PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/bin:/home/pi/aerowake-mit/Controller/

. /lib/init/vars.sh
. /lib/lsb/init-functions
# If you need to source some other scripts, do it here

case "$1" in
  start)
    log_begin_msg "Starting mavproxy service"
    sudo /home/pi/aerowake-mit/bin/launch_mavproxy.sh
    log_end_msg $?
    exit 0
    ;;
  stop)
    log_begin_msg "Stopping the coolest service ever unfortunately"

    # do something to kill the service or cleanup or nothing
    killall mavproxy.py
    log_end_msg $?
    exit 0
    ;;
  *)
    echo "Usage: /etc/init.d/auto_start.sh {start|stop}"
    exit 1
    ;;
esac
