# Long-Range SSH (NanoStation + Bullet + Jetson)

## IP Map

| Host | IP |
|------|-----|
| Jetson | 192.168.1.50 |
| Bullet | 192.168.1.20 |
| NanoStation | 192.168.1.27 |
| Laptop (wired to NanoStation LAN) | 192.168.1.x (e.g. .100) |

## Wiring

- Laptop → splitter → PoE injector → NanoStation
- Jetson → PoE injector → Bullet

## Jetson: Configure Ethernet (enP8p1s0)

On the Jetson, run:

```bash
sudo nmcli connection add type ethernet ifname enP8p1s0 con-name direct-eth ipv4.method manual ipv4.addresses 192.168.1.50/24
sudo nmcli connection modify direct-eth connection.autoconnect yes
sudo nmcli connection up direct-eth
```

Verify: `ip -br addr show enP8p1s0` → should show 192.168.1.50/24

## Laptop: Sanity Checks

```bash
ping 192.168.1.27   # NanoStation
ping 192.168.1.20   # Bullet
ping 192.168.1.50   # Jetson
```

## SSH to Jetson (via Bullet jump host)

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa -J ubnt@192.168.1.20 lorenzo@192.168.1.50
```

## Map Access (SSH tunnel)

```bash
ssh -L 8080:localhost:8080 -J ubnt@192.168.1.20 lorenzo@192.168.1.50
```

Then open http://localhost:8080 in browser.
