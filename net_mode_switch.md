# EtherCAT / Network Mode Switching Guide

This guide explains how to switch a network interface between:

- **EtherCAT mode** (dedicated NIC, no IP traffic)
- **Normal mode** (standard Ethernet with IP/internet)

---

## 📌 Interface Used

EtherCAT NIC:
enp3s0

Wi-Fi (internet):
wlp0s20f3

---

## 🔁 Switching Modes

### ▶️ Switch to EtherCAT Mode

```bash
net_mode ethercat

What this does:

Disables NetworkManager control for enp3s0
Removes any IP address
Brings interface down (or isolates it)
Starts EtherCAT master
Disables NIC offloads (for better determinism)
🌐 Switch to Normal Network Mode
net_mode normal

What this does:

Stops EtherCAT master
Brings interface back up
Re-enables NetworkManager control
Attempts to reconnect / obtain IP
📊 Check Status
net_mode status

Shows:

Link state
IP address
NetworkManager state
EtherCAT master status
🔍 Manual Checks
Check Interfaces
ip -br link
Check IP Address
ip addr show enp3s0
Check NetworkManager
nmcli device status

Expected:

enp3s0 → disconnected (EtherCAT mode)
wlp0s20f3 → connected (internet)
⚙️ EtherCAT Status
ethercat master

Important fields:

Slaves: should match your setup (e.g. 6)
Link: must be UP
Tx/Rx frame rate: stable (e.g. 1000 Hz)
Lost frames: should NOT increase
🧪 Stability Test
ethercat master
sleep 10
ethercat master

Interpretation:

Same Lost frames → stable
Increasing Lost frames → timing/network issue
📈 Live Monitoring
watch -n 1 ethercat master

Watch for:

stable 1000 Hz cycle
no increase in lost frames
⚠️ Notes
EtherCAT mode disables IP networking on the NIC
Wi-Fi remains active (no loss of internet)
No reboot required when switching modes
Lost frames is cumulative (does NOT reset automatically)
🧠 Best Practice
Use dedicated NIC for EtherCAT
Keep internet on Wi-Fi or second NIC
Avoid mixing standard Ethernet traffic with EtherCAT
🛠 Troubleshooting
Interface not found
ip -br link

Update your script with the correct interface name.

No IP after switching to normal mode
nmcli device connect enp3s0
EtherCAT not attaching

Check:

ethercat master

Ensure:

interface is NOT managed by NetworkManager
no IP address assigned
correct interface in /etc/sysconfig/ethercat
Check if NIC is used by EtherCAT
ethercat master

Look for:
Main: <MAC> (attached)

🔧 Optional Performance Tweaks

Disable NIC offloads (important for generic driver):

sudo ethtool -K enp3s0 rx off tx off tso off gso off gro off lro off sg off

Pin IRQs and CPU cores (advanced tuning):

EtherCAT → isolated core
ROS2 → separate core
Avoid shared IRQs
📊 Mode Summary
Mode	NIC State	IP	EtherCAT	Use Case
EtherCAT	unmanaged/down	❌	✅	Real-time control
Normal	managed/up	✅	❌	Internet / debugging
✅ Example Workflow
# Switch to EtherCAT
net_mode ethercat

# Verify
net_mode status

# Run ROS2 + control
ros2 launch ...

# Monitor
watch -n 1 ethercat master

# Switch back to normal
net_mode normal
🚀 Summary
Switching is safe and reversible
No reboot required
Your setup is correctly isolated
Monitor Lost frames for real-time stability