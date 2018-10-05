# start/stop all screens and processes

alias preflight.sh='sh ~/creare_ws/src/aerowake/aerowake_git/preflight.sh'
alias postflight.sh='sh ~/creare_ws/src/aerowake/aerowake_git/postflight.sh'

# copy camera calib to appropriate locations

alias copy-calib-to-aerowake='sh ~/creare_ws/src/creare/scripts/copy-calib-to-aerowake.sh'

# fast shutdown and reboot

alias shutdown='sudo shutdown now -P'
alias reboot='sudo reboot now'

# fast screen actions, start, detach, and reattach

alias sls='screen -ls'
alias skl='killall screen'

alias ssr='screen -S roscore'
alias sdr='screen -d roscore'
alias srr='screen -r roscore'

alias ssy='screen -S yaw-cmd'
alias sdy='screen -d yaw-cmd'
alias sry='screen -r yaw-cmd'

alias ssf='screen -S flight-cmd'
alias sdf='screen -d flight-cmd'
alias srf='screen -r flight-cmd'

alias ssb='screen -S bags'
alias sdb='screen -d bags'
alias srb='screen -r bags'

alias ssa='screen -S airprobe'
alias sda='screen -d airprobe'
alias sra='screen -r airprobe'

alias ssal='screen -S probe-check'
alias sdal='screen -d probe-check'
alias sral='screen -r probe-check'
