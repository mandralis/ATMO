# Create a new tmux session named "local"
tmux new-session -d -s local

# Split the window into 3 panes
tmux split-window -h
tmux split-window -h

# Select pane 0
tmux select-pane -t 0

# launch optitrack node
tmux send-keys -t local:0.0 './launch_opti.sh' C-m

# Select pane 1
tmux select-pane -t 1

# launch relay_mocap node
tmux send-keys -t local:0.1 './launch_relay.sh' C-m

# Select pane 2
tmux select-pane -t 2

# run plotjuggler
tmux send-keys -t local:0.2 './launch_plot.sh' C-m


