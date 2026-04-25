#!/usr/bin/env python3
"""
DIAGNOSTIC TOOL: Identify why swing-up fails

Langkah-langkah sistematis:
1. Check apakah cart bisa bergerak
2. Check apakah energi dihitung dengan benar
3. Check berapa sudut maksimal dicapai
"""

print("""
╔═══════════════════════════════════════════════════════════════════════════╗
║     SWING-UP DIAGNOSTIC - LANGKAH DEMI LANGKAH                           ║
╚═══════════════════════════════════════════════════════════════════════════╝

STEP 1: VERIFY CART MOVEMENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

COMMAND di-publish ke: /pendulum/cart_velocity_cmd (ROS2)
Destination di Gazebo: /model/linear_inverted_pendulum/joint/cart_slider/cmd_vel

⚠️  KEMUNGKINAN ISSUE #1:
   Bridge config di sim.launch.py:
   
   /model/linear_inverted_pendulum/joint/cart_slider/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double
   
   Ini CORRECT untuk Gazebo Harmonic dengan JointController plugin.
   Tapi perlu diverifikasi apakah plugin read dari topic ini.

✅  FIX: Cek URDF apakah ada actuator/effort controller setup


STEP 2: MINIMUM ENERGY REQUIREMENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Untuk swing-up dari θ = 0 (bawah) ke θ = 180° (atas):

Energy required = m*g*L * (1 + cos(θ_final)) - m*g*L * (1 + cos(0))
                = 0.20 * 9.81 * 0.20 * (1 + cos(180°)) - 0.20 * 9.81 * 0.20 * 2
                = 0.20 * 9.81 * 0.20 * 0 - 0.784
                = -0.784 J
                
Wait, that's NEGATIVE yang berarti energy TURUN di posisi atas!

❌ ERROR DI ENERGY CALCULATION!

Sebenarnya, untuk pendulum:
- θ = 0 (hanging down)  → PE = 0
- θ = π (standing up)   → PE = m*g*L * 2  (maksimal!)

Energy naik dari bawah ke atas = GAIN 2*m*g*L

Tapi di kode kami calculatenya:
  PE = m*g*L*(1 + cos(θ_from_vertical))
  
θ_from_vertical = θ - π
  
Jadi:
- θ = 0 (down):   θ_from_vertical = -π,  PE = m*g*L*(1 + cos(-π)) = m*g*L*(1-1) = 0 ✓
- θ = π (up):     θ_from_vertical = 0,   PE = m*g*L*(1 + cos(0)) = m*g*L*2 ✓

Rumus betul! Tapi swing-up mungkin tidak cukup power.


STEP 3: AMPLITUDO & FREQUENCY CALCULATION  
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Current swing-up usar simple sinusoidal oscillation:
  x(t) = A * sin(ωt)
  
Dengan:
  A = 0.25 + 0.13 * (E/E_target)  → A bervariasi dari 0.25 ke 0.38 m
  ω = 0.75 Hz  → Period = 1.33 sec
  
Implication:
  - Amplitude mulai 0.25m, sangat KECIL relatif ke rail ±0.39m
  - Velocity = A*ω*cos(ωt) → max = 0.38 * 2π * 0.75 ≈ 1.79 m/s
  - Tapi velocity limit = 1.5 m/s → COMMAND AKAN SATURATE!

🔴 PROBLEM: Control output saturated!


STEP 4: WHAT ACTUALLY HAPPEN SAAT SWING-UP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. START swing-up (X button)
2. Bridge mengirim velocity command target_x * ω * P_gain
3. Gazebo JointController membaca command dan set effort
4. Cart mulai bergerak (hopefully!)
5. Pendulum mulai swing
6. Energy naik, amplitude meningkat
7. Ketika angle > 20° → switch to BALANCE mode

MASALAHNYA:
- Jika cart bergerak TERLALU LAMBAT atau TIDAK SAMA SEKALI
  → Energi tidak naik
  → Pendulum tidak swing tinggi

- Jika cart saturated (velocity limit)
  → Kontrol tidak linear
  → Swing-up tidak stabil


STEP 5: DEBUGGING CHECKLIST
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[ ] 1. Check apakah cart bergerak:
        ros2 topic echo /joint_states | grep cart_slider
        → Lihat position berubah atau stuck?

[ ] 2. Check apakah command dikirim:
        ros2 topic echo /pendulum/cart_velocity_cmd
        → Lihat nilai command berubah saat swing-up?

[ ] 3. Check energi calculation:
        ros2 topic echo /pendulum/sim_state
        → Lihat element [3] (energi) naik?

[ ] 4. Check angle progress:
        ros2 topic echo /pendulum/sim_state
        → Lihat element [0] (degree) berubah?

[ ] 5. Cek console sim_serial_bridge:
        Lihat output "SWING θ=..." untuk debug log?
        
[ ] 6. Cek console Gazebo:
        Ada error message?


HIPOTESIS PALING LIKELY:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔴 CART TIDAK BERGERAK
   → Bridge tidak working
   → Velocity command tidak delivery ke Gazebo
   → Perlu FIX di launch file atau URDF


SOLUSI CEPAT:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Jika cart tidak bergerak:
1. Tambah effort controller plugin ke URDF (bukan velocity)
2. Ubah swing-up untuk generate effort command
3. Atau fix bridge configuration


NEXT ACTION:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Run manual velocity test (test_cart_movement.py)
2. Monitor console output
3. Report findings
4. Adjust based on what's broken
""")
