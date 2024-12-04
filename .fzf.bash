# Setup fzf
# ---------
if [[ ! "$PATH" == */home/dockerian/.fzf/bin* ]]; then
  PATH="${PATH:+${PATH}:}/home/dockerian/.fzf/bin"
fi

eval "$(fzf --bash)"
