#!/usr/bin/env python3
"""
ANALYSIS: Why pendulum tidak bisa tegak di simulasi Gazebo
"""

import math

print("""
╔════════════════════════════════════════════════════════════════════════╗
║                  SWING-UP FAILURE ROOT CAUSE ANALYSIS                  ║
╚════════════════════════════════════════════════════════════════════════╝

📊 PHYSICAL CONSTRAINTS:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Pendulum mass:       m = 0.20 kg
Pendulum COM:        L = 0.20 m  
Gravitational:       g = 9.81 m/s²
Rail limit:          ±0.39 m
Cart mass:           M = 1.20 kg
Friction (cart):     f = 0.001  (VERY LOW)
Damping (cart):      d = 0.05   (VERY LOW)
Damping (pend):      d = 0.0008 (MINIMAL)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔴 POTENTIAL PROBLEMS:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1️⃣  SIMULATION PHYSICS ISSUE:
   - Gazebo Harmonic mungkin punya physics timestep yang sampai (solver)
   - Joint inertia/damping tidak cocok dengan parameter
   - Effort control mungkin tidak bisa reach velocity command yang diinginkan

2️⃣  COMMAND NOT REACHING GAZEBO:
   - Bridge dari /pendulum/cart_velocity_cmd ke Gazebo mungkin BROKEN
   - Gazebo tidak subscribe atau tidak exec velocity command
   - Command velocity ≠ Command force/effort

3️⃣  ALGORITHM ISSUES:
   - Swing-up formula tidak optimal untuk Gazebo physics
   - Proportional gain 4.0 terlalu tinggi → oscillation
   - Amplitude 0.25m tidak cukup untuk pump energy
   - Energy calculation mungkin wrong (using potential + kinetic)

4️⃣  CONTROL GAIN MISMATCH:
   - K value dihitung asli untuk hardware STM32
   - Gazebo dynamics BERBEDA dari real hardware
   - Gain linear 189.1 PWM/cm/s → 100 PWM/cm/s tapi masih anggapan
   - Sebenarnya Gazebo pake unit physics, bukan PWM

5️⃣  FUNDAMENTAL ISSUE:
   ⚠️ Bridge itu subscribe /pendulum/cart_velocity_cmd 
      dan publish ke /model/linear_inverted_pendulum/joint/cart_slider/cmd_vel
   
   Tapi Gazebo mungkin EXPECT effort command, bukan velocity!
   Joint controller mungkin ada di config yang SALAH

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ SOLUTION STRATEGY:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Step 1: FIX GAZEBO CONFIG
   ✓ Verify joint controller teknis di URDF
   ✓ Check apakah cart_slider bisa gerakan
   ✓ Validate damping/friction ratio

Step 2: TEST CART MOVEMENT DIRECTLY
   ✓ Publish velocity command manual ke /pendulum/cart_velocity_cmd
   ✓ Monitor /joint_states untuk lihat cart_slider.position berubah
   ✓ Jika tidak berubah → BRIDGE BROKEN

Step 3: SIMPLIFY SWING CONTROL
   ✓ Tidak pakai energy shaping (complex)
   ✓ Pakai pure sinusoidal + P control (simple)
   ✓ Increase amplitude & frequency

Step 4: RECALIBRATE GAINS
   ✓ Dari test real pendulum, ambil empirical data
   ✓ Adjust K untuk simulasi Gazebo
   ✓ Tune di-GUI sampai stable

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")

# Energy calculation
m = 0.20
L = 0.20
g = 9.81

print("\n📐 ENERGY CALCULATION:")
print(f"Target energy Etarget = 2×m×g×L = 2×{m}×{g}×{L} = {2*m*g*L:.3f} J")
print()

# Test case: pendulum at different angles
angles_deg = [0, 30, 60, 90, 120, 150, 180]
print("     Angle | Potential | Kinetic | Total | % Progress")
print("    -------|-----------|---------|-------|------------")
for deg in angles_deg:
    rad = math.radians(deg)
    # Potential energy (assuming θ=0 is down, θ=π is up)
    # PE = m*g*L*(1 + cos(θ))
    theta_from_vertical = rad - math.pi  # Convert to from-vertical angle
    PE = m * g * L * (1.0 + math.cos(theta_from_vertical))
    KE = 0  # Assuming at peak (v=0)
    total = PE + KE
    pct = (total / (2*m*g*L)) * 100
    print(f"    {deg:3d}° | {PE:9.3f} | {KE:7.3f} | {total:5.3f} | {pct:5.1f}%")

print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

CONCLUSION:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Masalah paling kemungkinan:

  🔴 CART TIDAK BERGERAK!
     → Bridge command tidak bekerja
     → Perlu test manual velocity publish
     → Verifikasi Gazebo joint controller setup

Atau:

  🔴 SWING-UP TIDAK PUMP ENERGY CUKUP
     → Amplitude terlalu kecil (0.25-0.38m dari total ±0.39m limit)
     → Frequency 0.75Hz terlalu lambat
     → Gain 4.0 tidak cukup untuk Gazebo friction
""")
