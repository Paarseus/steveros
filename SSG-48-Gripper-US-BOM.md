# SSG-48 Adaptive Electric Gripper - US-Sourced BOM

> **Original Project:** [PCrnjak/SSG-48-adaptive-electric-gripper](https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper)
> **Docs:** [source-robotics.github.io/SSG48-gripper-docs](https://source-robotics.github.io/SSG48-gripper-docs/page1_about_the_gripper/)
> **Specs:** 48mm stroke, 400g, 5-80N grip force, 12-24V, CAN bus

---

## 1. BLDC Gimbal Motor - GBM5208-75

The SSG-48 uses a **GBM5208-75** brushless gimbal motor (24N22P, 11 pole pairs, hollow shaft, ~189g).

| Option | Seller | Price | Link |
|--------|--------|-------|------|
| **GBM5208-75 (exact match)** | Amazon US | ~$30-50 | [Amazon B0DGL9XJ14](https://www.amazon.com/GBM5208-75-brushless-Camera-Working-Characteristics/dp/B0DGL9XJ14) |
| GBM5208-75 (variant listing) | Amazon US | Check | [Amazon B0FW4K2S9S](https://www.amazon.com/GBM5208-75-brushless-Camera-Working-Characteristics/dp/B0FW4K2S9S) |
| GBM5208-75T 80KV | RCTimer | $39.99 | [rctimer.com](https://www.rctimer.com/rctimer-gimbal-motor-gbm5208-75t-p0448.html) |
| DYS GBM5208-75 | Off The Grid Sun | $56.89 | [offthegridsun.com](https://offthegridsun.com/OFFTHEGRIDWATER.CA/BRUSHLESS-GIMBAL-PTZ/Gimbal-Brushless-Motor/DYS-GBM5208-75-Gimbal-Brushless-Motor) |
| iPower GM5208-24 (alt winding) | RobotShop | Check | [RobotShop](https://www.robotshop.com/products/ipower-motor-gm5208-24-brushless-gimbal-motor) |

**Recommendation:** Amazon GBM5208-75 or RCTimer ($39.99) for exact match. The GM5208-24 is same size/pole count but different winding (higher speed, less torque).

---

## 2. BLDC Motor Driver - Spectral Micro

The Spectral Micro is made by Source Robotics (Croatia). It has FOC control, 14-bit encoder, CAN bus, 12-28V, 2.8A continuous.

| Option | Seller | Price | Link |
|--------|--------|-------|------|
| **Spectral Micro (original)** | Source Robotics (ships to US) | ~$93 | [source-robotics.com](https://source-robotics.com/products/spectral-micro-bldc-controller) |
| Spectral Micro | Tindie (often out of stock) | $83 | [Tindie](https://www.tindie.com/products/sourcerobotics/spectral-micro-bldc-controller/) |
| **ODrive Micro (US alt)** | ODrive Robotics (USA) | $89 | [ODrive Shop](https://shop.odriverobotics.com/products/odrive-micro) |
| **moteus r4.11 (US alt)** | mjbots (USA) | $79 | [mjbots.com](https://mjbots.com/products/moteus-r4-11) |

**Recommendation:** Buy the **Spectral Micro directly from Source Robotics** for guaranteed firmware compatibility with the SSG-48. If you want US-only, the **ODrive Micro ($89)** or **moteus r4.11 ($79)** are excellent alternatives but will require custom firmware integration.

---

## 3. Linear Rails & Carriages

| Part | Qty | Seller | Price | Link |
|------|-----|--------|-------|------|
| **MGN7 100mm rail + 2x MGN7C blocks (combo)** | 1 set | Amazon US | ~$14-18 | [Amazon B01IMVIGHM](https://www.amazon.com/2Sets-Linear-Guide-Length-Carriage/dp/B01IMVIGHM) |
| MGN7 100mm rail + 1 MGN7C block | 1 | Amazon US | ~$8-13 | [Amazon B0CJYMCG1M](https://www.amazon.com/Miniature-Extended-Package-Printers-Machines/dp/B0CJYMCG1M) |
| Mssoomm MGN7C blocks only (2-pack) | 1 | Amazon US | ~$8-12 | [Amazon B08ZS6DFCN](https://www.amazon.com/Mssoomm-Carriage-Miniature-Sliding-Guideway/dp/B08ZS6DFCN) |

**Recommendation:** Get the **2-set combo (B01IMVIGHM)** - includes 2 rails + 2 MGN7C carriages, covers everything in one purchase.

---

## 4. Shaft Coupler

| Part | Qty | Seller | Price | Link |
|------|-----|--------|-------|------|
| **uxcell 8mm-to-8mm rigid coupling, M4 screws** | 1 | Amazon US | ~$5-8 | [Amazon B07P6X92BT](https://www.amazon.com/uxcell-Coupling-L22xD14-Coupler-Connector/dp/B07P6X92BT) |
| uxcell 8mm aluminum rigid coupling, M4 screws | 1 | Amazon US | ~$5-7 | [Amazon B07P82D737](https://www.amazon.com/uxcell-Coupling-L25xD14-Stepper-Coupler/dp/B07P82D737) |

---

## 5. Magnet

| Part | Qty | Seller | Price | Link |
|------|-----|--------|-------|------|
| **6mm x 2.5mm N38 neodymium disc** | 1 | AMF Magnets USA | $0.70 | [AMF Magnets](https://amfmagnets.com/products/neodymium-disc-6mm-x-2-5mm-n38-cu) |
| 6mm x 2.5mm N52 disc (50-pack) | 1 | Magnet Baron (US) | $16.99/50 | [Magnet Baron](https://themagnetbaron.com/products/50pcs-6mm-x-2-5mm-1-4-x-3-32-disc-magnets) |

**Recommendation:** **AMF Magnets USA** at $0.70 for a single magnet.

---

## 6. Screws

| Screw | Qty | Seller | Price | Link |
|-------|-----|--------|-------|------|
| M2.5 x 8mm SHCS | 4 | Amazon US | ~$3-4 | [Amazon B0006NAN4O](https://www.amazon.com/Du-Bro-2117-Socket-Screw-4-Pack/dp/B0006NAN4O) |
| M2 x 4mm SHCS | 8 | Amazon US | ~$6/25pk | [Amazon B0765L5QXT](https://www.amazon.com/M2-0-4-Black-Oxide-Steel-Socket/dp/B0765L5QXT) |
| M2 x 10mm SHCS | 4 | Amazon US | ~$6/10pk | [Amazon B004NEH0LQ](https://www.amazon.com/Metric-10mm-Socket-Screw-Black/dp/B004NEH0LQ) |
| **M3 x 5mm cheese head** | 5 | Accu (US) | ~$0.10/ea | [Accu](https://accu-components.com/us/slotted-cheese-head-screws/6498-SFE-M3-5-A2) |
| **M3 x 8mm cheese head** | 7 | Accu (US) | ~$0.10/ea | [Accu](https://accu-components.com/us/slotted-cheese-head-screws/6500-SFE-M3-8-A2) |
| M3 x 10mm SHCS | 4 | Amazon US | ~$6/50pk | [Amazon B08Q87982W](https://www.amazon.com/Socket-Screws-Grade-Alloy-Thread/dp/B08Q87982W) |
| M3 x 12mm SHCS | 4 | Amazon US | ~$7/100pk | [Amazon B07KRRNDGC](https://www.amazon.com/gp/product/B07KRRNDGC) |
| M4 x 10mm SHCS | 4 | Amazon US | ~$6/10pk | [Amazon B00C1GJ2HS](https://www.amazon.com/Metric-10mm-Socket-Screw-Black/dp/B00C1GJ2HS) |

**Note:** The cheese head screws (M3x5 and M3x8) are a specific DIN 84 style. **Accu Components US** sells them individually with no minimum order. McMaster-Carr is another option.

---

## 7. Brass Heat-Set Inserts

| Insert | Qty | Seller | Price | Link |
|--------|-----|--------|-------|------|
| M3 x 3mm length, 4.5mm OD | 1 | eBay (US seller) | ~$3-5 | [eBay 295558271926](https://www.ebay.com/itm/295558271926) |
| **M3 x 5mm length, 4.5mm OD** | 15 | Partsbuilt 3D | ~$2/4pk | [Partsbuilt](https://partsbuilt.com/m3-heat-set-thread-insert-5mm-long-4-pack/) |
| M3 x 5mm length, 4.5mm OD | 15 | eBay (US seller) | ~$3-6 | [eBay 295558272127](https://www.ebay.com/itm/295558272127) |
| M3 heat-set inserts (various) | -- | Amazon US | varies | [Amazon search](https://www.amazon.com/s?k=m3+heat+set+inserts) |

**Note:** You need 15 of the M3x5mm inserts. Partsbuilt sells 4-packs, so order 4 packs. Alternatively, CNCKitchen's standard insert (5.7mm length) may work if 0.7mm extra length is acceptable.

---

## 8. Connectors & Cables

| Part | Qty | Seller | Price | Link |
|------|-----|--------|-------|------|
| **XH 2.54mm 2-pin cable, 300mm** | 1 | Amazon US | ~$9/80pk | [Amazon B00L44CENM](https://www.amazon.com/2-54mm-Pitch-Female-Connector-26AWG/dp/B00L44CENM) |
| **ZH 1.5mm 2-pin cable, 300mm** | 1 | Amazon US | ~$7-9 | [Amazon B0FZVMNZR6](https://www.amazon.com/BHUPWZE-Connector-Length-Double-Head_10PCS_5P/dp/B0FZVMNZR6) |
| **M8 4-pin male panel-mount** | 1 | Digi-Key | ~$4-8 | [Digi-Key (Amphenol LTW)](https://www.digikey.com/en/products/detail/amphenol-ltw/M8S-04PMMR-SF8001/7618869) |
| **M8 4-pin male wire connector** | 1 | Amazon US | ~$8-10 | [Amazon (Elecbee)](https://www.amazon.com/Elecbee-Connector-Straight-Aviation-Screw-Joint/dp/B07SS8TG2W) |
| **M8 4-pin female wire connector** | 1 | Amazon US | ~$8-10 | [Amazon (Elecbee)](https://www.amazon.com/Elebee-Wireable-Connector-Shielded-Screw-Joint/dp/B07T1YLXK5) |
| **26AWG 4-wire twisted pair, 0.4m** | 1 | Amazon US | ~$3-5 | Buy a short Cat5e patch cable and strip the outer jacket |

---

## 9. 3D Printed Parts

- **Material:** PETG
- **STL/STEP files:** [GitHub repo](https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper)
- **Also on:** [Thingiverse](https://www.thingiverse.com/thing:6606093), [Printables](https://www.printables.com/model/840628), [MakerWorld](https://makerworld.com/en/models/451988)

---

## Shopping Strategy (Minimize Orders)

| Store | What to Buy | Est. Total |
|-------|-------------|------------|
| **Amazon US** | Motor, linear rails, shaft coupler, all SHCS screws, XH/ZH cables, M8 wire connectors, Cat5e cable | ~$80-120 |
| **Source Robotics** | Spectral Micro BLDC driver (or starter kit) | ~$93 |
| **Accu Components US** | M3 cheese head screws (5mm + 8mm) | ~$2-3 |
| **AMF Magnets USA** | 6mm x 2.5mm neodymium magnet | ~$0.70 |
| **Digi-Key** | M8 panel-mount connector | ~$4-8 |
| **eBay / Partsbuilt 3D** | M3 heat-set inserts (4.5mm OD) | ~$10-15 |
| | **Estimated Total (excl. 3D printing)** | **~$190-240** |

---

## Resources

- [Assembly Manual (PDF)](https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper/tree/main/Assembly%20manual)
- [Hackaday Project Page](https://hackaday.io/project/202979-ssg-48-adaptive-electric-gripper)
- [Hackster.io Page](https://www.hackster.io/Source_robotics/ssg-48-3d-printed-adaptive-electric-gripper-73fb3f)
- [SSG Gripper GUI (Python)](https://github.com/PCrnjak/SSG-gripper-GUI)
- [Spectral BLDC Docs](https://source-robotics.github.io/Spectral-BLDC-docs/)
