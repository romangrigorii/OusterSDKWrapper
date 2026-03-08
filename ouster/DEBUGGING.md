# Debugging Ouster connection (Timeout / no data)

If you see **"Timeout was reached"** when capturing or **no points** in live view, follow these steps.

---

## Why "Ethernet: 169.254.x.x" means the sensor is on 169.254.x.x

If your **Ethernet** adapter shows **169.254.x.x** (e.g. 169.254.74.254), that is **link-local**: there is no DHCP on that cable, so Windows and the Ouster both self-assign from 169.254.0.0/16. So the **sensor is not** at 172.17.1.56 — it is at some **169.254.x.x** address. The default config (172.17.1.56) is for a network where you ran a script that set the PC to 172.17.1.100. With a direct cable, use the IP that `--check` finds (or try `--sensor-ip 169.254.74.1` if your PC Ethernet is 169.254.74.254).

---

## 1. Run the built-in check

From the `LidarSDKWrapper/ouster` folder:

```powershell
python setup_ouster.py --check
```

If your sensor is not at the default IP (172.17.1.56):

```powershell
python setup_ouster.py --check --sensor-ip 192.168.1.100
```

This prints:
- Your PC’s IPv4 addresses (to confirm subnet)
- **If Ethernet is 169.254.x.x:** a short explanation and automatic probing of common sensor IPs (e.g. 169.254.74.1); if one responds, it tells you exactly which `--sensor-ip` to use
- Ping to the sensor
- HTTP request to the sensor (metadata API)
- Firewall reminder

---

## 2. Put the PC on the same subnet as the sensor

- **Windows:** Run the network script (configures **Ethernet only**, not Wi‑Fi):
  ```powershell
  .\setup_ouster_network.ps1
  ```
  Connect the Ouster with an **Ethernet cable** to the PC; the script sets the Ethernet adapter to 172.17.1.100.

- **Manual:** Set the **Ethernet** adapter (not Wi‑Fi) to a static IP in the sensor’s subnet, e.g.:
  - Sensor 172.17.1.56 → PC Ethernet 172.17.1.100, mask 255.255.255.0
  - Or use whatever IP you got from `--discover`.

---

## 3. Find the sensor IP (if unknown)

```powershell
python setup_ouster.py --discover
```

Use the IP it prints with `--sensor-ip`:

```powershell
python setup_ouster.py --live --sensor-ip <IP>
python setup_ouster.py --check --sensor-ip <IP>
```

---

## 4. Check firewall

The PC must receive **UDP** on:

- Lidar: **7510** (default)
- IMU: **7511** (default)

**Windows:** Windows Defender Firewall → Advanced settings → Inbound Rules → New Rule → Port → UDP → 7510, 7511 → Allow.

---

## 5. Sensor must send data to the PC

The Ouster sends UDP to the **destination IP/port** it is configured with. After you set the PC’s Ethernet to 172.17.1.100, the sensor must be configured to send to 172.17.1.100:7510 (and 7511 for IMU). The `setup_ouster_network.ps1` script tries to set this via the sensor HTTP API when the sensor is reachable. If the sensor was previously configured for another host, reconfigure it (e.g. run the network script again once the PC can reach the sensor over HTTP).

---

## Quick checklist

| Step | Action |
|------|--------|
| 1 | Run `python setup_ouster.py --check` (and `--sensor-ip <IP>` if needed). |
| 2 | Connect Ouster to PC with **Ethernet**. Run `.\setup_ouster_network.ps1` (Ethernet only). |
| 3 | Run `python setup_ouster.py --discover` to see the sensor IP; use it with `--sensor-ip` if different from 172.17.1.56. |
| 4 | Allow UDP 7510 and 7511 in Windows Firewall (inbound). |
| 5 | Run `python setup_ouster.py --live` (add `--sensor-ip <IP>` if not default). |

If `--check` shows ping and HTTP OK but live view still has no points, the sensor is likely not sending UDP to this PC’s IP/port; reconfigure the sensor’s UDP destination to your PC’s Ethernet IP and ports 7510/7511.
