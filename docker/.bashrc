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
# Set the prompt
PS1='\[\033[01;32m\]\u@docker:\[\033[01;34m\]\w\[\033[00m\]\$ '

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

# Some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# ROS2 aliases
alias cb='colcon build && source install/setup.bash'
alias cbp='colcon build --symlink-install --packages-select'
alias cbt='colcon build --symlink-install --packages-select --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias run='ros2 run'
alias launch='ros2 launch'
alias topic='ros2 topic'
alias node='ros2 node'
alias param='ros2 param'


[ -f ~/.fzf.bash ] && source ~/.fzf.bash
