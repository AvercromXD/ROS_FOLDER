# .bashrc
export HISTFILE=~/.bash_history
# Source ROS setup files
source /opt/ros/humble/setup.bash

# Path to the Default ROS2 Workspace
ROS2_WS="turtlebot3_ws"
USER="dockerian"

if [ -f "/home/$USER/$ROS2_WS/install/setup.bash" ]; then
    source "/home/$USER/$ROS2_WS/install/setup.bash"
fi

# Enable color support for ls and add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

export ROS_DOMAIN_ID=37
export TURTLEBOT3_MODEL=custom



[ -f ~/.fzf.bash ] && source ~/.fzf.bash
