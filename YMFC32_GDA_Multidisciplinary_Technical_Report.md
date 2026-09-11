**Generalized Dimensional Analysis and a Multidisciplinary\
Reduced-Order Digital Design Model for the Open-Source YMFC-32
Quadcopter**

*Technical Report*

Dr. Yi-Kuen Lee

Department of Mechanical and Aerospace Engineering, HKUST

Prepared for postgraduate study, model-based design, and experimental
calibration

**Abstract---**This report develops and validates a compact,
multidisciplinary design model for the open-source YMFC-32 quadcopter.
The methodology begins with Generalized Dimensional Analysis (GDA), uses
Buckingham-Pi groups to expose the governing similarity coordinates, and
then closes the dimensionless structure with rotor momentum theory,
rigid-body mechanics, battery-energy balance, sampled-data control,
first-order actuator dynamics, and a single-mode structural/IMU
vibration model. The most important performance outputs are maximum
thrust, drag, hover power, endurance, altitude capability, allowable
payload, control sampling adequacy, vibration separation, and compute
utilization. A key validation distinction is maintained throughout: the
250 g/5-inch case used in the project GDA is a design sandbox, not the
published YMFC-32 airframe. Public YMFC-32 data instead support a
450-class platform, including a documented 1.04 kg, 9.4-inch-propeller
build that flew for 23 min. Using that aircraft as an aggregate
reference, the momentum-theory model predicts about 127.8 W while
nominal battery energy divided by reported flight time gives about 127.4
W, which supports the power scaling at the system level but does not
independently identify figure of merit, motor efficiency, or usable
battery fraction. The final result is a reduced-order digital design
kernel implemented consistently in MATLAB and Python and organized so
that uncertain parameters can be replaced by measured values.

**Index Terms---**YMFC-32, quadrotor, dimensional analysis, Buckingham
Pi theorem, rotor momentum theory, rigid-body dynamics, PID control,
sampled-data control, MEMS IMU, vibration, payload, endurance,
reduced-order model.

# I. INTRODUCTION AND SCOPE

A useful engineering model should be simple for the right reason. If the
model is simple because an important physical mechanism was forgotten,
it is dangerous; if it is simple because the dominant mechanisms have
been isolated and nondimensionalized, it is powerful. The YMFC-32 is a
particularly good teaching platform because it is not merely an aircraft
and not merely a controller. It is a closed loop in which the propellers
accelerate air, the rigid body responds, the frame vibrates, a MEMS
inertial sensor measures that motion, software estimates attitude, and a
digital controller sends new commands to the motors.

The project source material identifies the YMFC-32 as an
STM32F103C8T6-based flight controller with an MPU-6050 IMU, a
deterministic 250 Hz (4 ms) loop, complementary filtering, cascaded
angle-to-rate PID control, and Quad-X motor mixing. The GDA work
decomposes the problem into rotor/airframe, attitude-control, and
compute/thermal passes and introduces the key groups C_T, C_P, J, Pi_TW,
Pi_I, Re_rotor, Ma_tip, S=f_L/f_n, damping ratio, normalized PID gains,
and compute utilization.

Public documentation from Joop Brokking gives a different physical scale
from the original GDA sandbox: the recommended base hardware is a
450-size frame with four 1000 kV motors and 10x4.5 propellers, with
Brokking stating a preference for 8x4.5 propellers for lower motor/ESC
loading and faster response \[1\]. This distinction matters because
dimensional similarity is not the same thing as geometric resemblance.

![](media/image1.png){width="2.729899387576553in"
height="1.6131233595800525in"}

Fig. 1. Multidisciplinary reduced-order design loop. The arrows
emphasize that propulsion, mechanics, sensing, control, and computing
form one closed physical-information system.

# II. PHYSICAL PICTURE BEFORE THE ALGEBRA

Start with the simplest question: what keeps the vehicle in the air? The
four rotors must give the surrounding air downward momentum. The
reaction on the rotor disks is upward thrust. In hover the total upward
thrust equals weight. Every refinement in the model - finite propeller
efficiency, body drag, altitude, actuator lag, sensor vibration -
modifies how difficult it is to satisfy that simple balance, but it does
not replace the balance.

T_h = m g (1)

Equation (1) is more than bookkeeping. It tells us immediately why
payload is expensive: payload increases the force that the rotors must
produce continuously. It also increases inertia, so the controller has
more difficulty changing angular motion. Thus one kilogram added to a
quadrotor is not merely one kilogram added to a scale; it changes both
the energy problem and the control problem.

# III. GENERALIZED DIMENSIONAL ANALYSIS FRAMEWORK

Dimensional analysis says that a physically meaningful law can be
written in terms of dimensionless combinations of the relevant
variables. For n variables whose dimensional matrix has rank r,
Buckingham\'s theorem gives n-r independent dimensionless products. The
theorem reduces the number of independent coordinates, but it does not
supply the function connecting them. That function must come from
physics, computation, or experiment \[7\].

F(q_1, q_2, \..., q_n) = 0 -\> Phi(Pi_1, Pi_2, \..., Pi\_(n-r)) = 0 (2)

For YMFC-32 the useful GDA strategy is therefore \'one pass per
coupling, then assemble.\' Rotor aerodynamics uses M-L-T variables;
attitude control introduces time-scale ratios; compute enters as another
nondimensional constraint. The project GDA further uses an exponent
matrix to expose logarithmic sensitivities and, when needed,
singular-value decomposition to identify dominant design directions.

  ------------------------------------------------------------------------
  **Group**              **Definition**          **Physical question**
  ---------------------- ----------------------- -------------------------
  Pi_TW                  T_max/(m g)             Is there enough thrust
                                                 and control reserve?

  J                      V/(n D)                 How far is the rotor from
                                                 static/hover operation?

  Re_rotor               rho n D\^2/mu           Is viscous similarity
                                                 preserved?

  Ma_tip                 pi n D/c                Are tip compressibility
                                                 effects important?

  Pi_I                   I/(m R\^2)              How is mass distributed
                                                 relative to size?

  S                      f_L/f_n                 Is the controller sampled
                                                 fast enough?

  U_c                    C_loop f_L/f_CPU        Does the processor finish
                                                 the loop on time?
  ------------------------------------------------------------------------

# IV. ROTOR FLUID MECHANICS AND LIFT

## A. Static Thrust

For one rotor the dimensional structure of static thrust is T_r = C_T
rho n\^2 D\^4. Here C_T is not a universal constant; it represents the
propeller geometry and depends on operating regime through J, Reynolds
number, Mach number, blade geometry, and stall state.

T_max = N_r C_T rho n_max\^2 D\^4 (3)

For a four-rotor vehicle, N_r=4. The fourth power on diameter is
striking, but it should not be read as permission to make the propeller
arbitrarily large. Larger propellers increase aerodynamic leverage while
simultaneously changing motor current, rotor inertia, response time,
structural clearance, and C_T itself.

Pi_TW = T_max / (m g) (4)

The ratio Pi_TW is the cleanest first answer to \'can it fly?\' Pi_TW=1
only means static force balance at maximum thrust; it leaves no maneuver
margin. The project model therefore uses a design reserve Pi_TW,min=1.5
as an engineering screening criterion, not as a universal law.

## B. Hover Speed and Rotor Headroom

n_h = sqrt\[ m g / (N_r C_T rho D\^4) \] (5)

Equation (5) says that a heavier vehicle must hover at a higher rotor
speed. The rotor-speed fraction Pi_RPM=n_h/n_max is related to thrust
margin by Pi_RPM=Pi_TW\^(-1/2). At Pi_TW=1.5, the hover speed is already
about 81.6% of the modeled maximum speed. The remaining speed range is
what the controller must use to reject disturbances and command upward
acceleration.

# V. MOMENTUM THEORY, POWER, AND ENDURANCE

Now ask where the power goes. In the ideal actuator-disk picture, a
rotor creates thrust by increasing the momentum flux of air through a
disk. Conservation of mass, momentum, and energy gives the induced
velocity in hover \[8\]. For four equal rotors with total disk area A_d
= pi D\^2, the result is:

v_i = sqrt\[ m g / (2 rho A_d) \] (6)

P_i = (m g) v_i (7)

The ideal power is not the electrical power from the battery. Real
blades have profile drag, swirl, tip losses, and nonuniform inflow;
motors and ESCs dissipate electrical power. A compact correction uses
rotor figure of merit FM and motor/ESC efficiency eta_m:

P_hover = (m g)\^(3/2) / \[ FM eta_m sqrt(2 rho pi) D \] + P_avionics
(8)

The physics hiding inside Eq. (8) is worth noticing. The mass appears as
m\^(3/2). A 10% increase in mass therefore costs roughly 15% more
idealized hover power, while a larger rotor diameter reduces induced
power approximately in inverse proportion to D, provided similarity is
not badly broken.

## A. Battery Energy and Hover Time

E_use = m_b e_b eta_u (9)

t_h = E_use / P_hover (10)

Here m_b is battery mass, e_b is pack-level specific energy, and eta_u
is the usable fraction. Battery mass appears twice in the problem: it
supplies energy but also adds weight. That is why simply installing a
larger battery does not increase endurance without limit.

t_h \~ m_b / (m_rest + m_b)\^(3/2) (11)

For the simplified hover model, differentiating Eq. (11) gives an
interior optimum m_b,opt = 2 m_rest. This is a scaling result, not a
recommendation to install that battery without checking current, voltage
sag, frame loading, and thrust margin.

# VI. DRAG AND FORWARD FLIGHT

F_D = (1/2) rho (C_D A) V\^2 (12)

P_D = F_D V = (1/2) rho (C_D A) V\^3 (13)

Drag force grows with V\^2, but drag power grows with V\^3. The extra
factor of V appears because power is force times velocity. This is the
reason a multirotor can feel reasonably efficient at low speed and then
become energetically expensive when pushed faster. The current GDA model
uses C_D A as a lumped drag area because no trustworthy YMFC-32
wind-tunnel value was found; it is therefore a calibration parameter.

# VII. ALTITUDE AS TWO DIFFERENT LIMITS

There is no single \'maximum height\' in the physics. Two different
questions must be asked. First: at what altitude does thin air remove
the required thrust reserve? Second: does the battery contain enough
energy to climb there?

rho(h) = rho_0 exp(-h/H) (14)

h_T,res = H ln\[ Pi_TW,0 / Pi_TW,min \] (15)

Equation (15) is logarithmic. That is an important warning against
fitting a simple power law to altitude ceiling. The energy-limited climb
estimate is:

P_climb \~= P_hover + m g V_c / eta_m (16)

h_E = E_use V_c / P_climb (17)

h_model = min(h_T,res, h_E) (18)

These equations produce a physical model ceiling, not a legal, safe, or
flight-tested ceiling. The original GDA sandbox predicts kilometer-scale
values, but no reliable public YMFC-32 maximum-altitude flight test was
found. Those numbers must therefore remain model outputs, not validated
aircraft ratings.

![](media/image2.png){width="3.15in" height="1.9005129046369205in"}

Fig. 2. Payload sweep for the 250 g/5-inch GDA sandbox baseline. This
figure illustrates scaling behavior; it is not a published YMFC-32
airframe specification.

# VIII. RIGID-BODY DYNAMICS AND PAYLOAD INERTIA

The translational equation is Newton\'s second law; the rotational
equation is its angular counterpart. Near hover, the latter is
especially important because the attitude controller spends most of its
effort creating torque, not total force.

m V_dot = F_thrust + F_drag + m g (19)

I omega_dot + omega x (I omega) = tau (20)

For small roll motion, I_x p_dot \~= tau_phi. A payload located near the
center of mass can add substantial weight with little roll/pitch
inertia, while the same mass placed far from the center adds I
approximately as m_p r_p\^2. Two quadrotors with the same total mass can
therefore need very different control gains.

I_new = I_0 + m_p r_p\^2 (21)

# IX. MOTOR DYNAMICS, SATURATION, AND CONTROL AUTHORITY

Static thrust is not enough to describe a controlled aircraft. The
motor-propeller system takes time to change speed. A first-order
actuator model captures the dominant lag without pretending to model
every winding, ESC commutation event, and blade transient.

tau_m omega_dot + omega = K_m u (22)

The dimensionless actuator ratio 2 pi f_BW tau_m compares controller
speed with motor speed. If this number is small, the actuator looks
nearly instantaneous. If it approaches unity, the actuator pole consumes
phase margin and aggressive PID tuning becomes fragile.

Pi_m = 2 pi f_BW tau_m (23)

Actuator saturation is equally fundamental. When collective throttle is
already high, one or more motors may have insufficient upward command
range to generate the requested roll or pitch torque. This is why a
useful payload limit must include actuator headroom and cannot be
inferred from T_max = m g alone.

# X. STRUCTURAL VIBRATION AND MEMS INERTIAL SENSING

An IMU does not measure an abstract rigid body; it measures the motion
of the small piece of circuit board on which it is mounted. If the frame
or mounting resonates, the sensor sees that vibration and the controller
may react to it as if the whole aircraft were rotating.

m_s x_ddot + c_s x_dot + k_s x = F_v(t) (24)

f_s = (1/2pi) sqrt(k_s/m_s), zeta_s = c_s/\[2 sqrt(k_s m_s)\] (25)

Dominant rotor forcing occurs near the one-per-revolution frequency
f_1P=n and the blade-passing frequency f_BPF=N_b n. The simplest
vibration screening coordinate is the ratio of forcing frequency to
structural natural frequency.

r = f/f_s (26)

T_v = sqrt{ \[1+(2 zeta_s r)\^2\] / \[(1-r\^2)\^2 + (2 zeta_s r)\^2\] }
(27)

When r is near one, resonance magnifies the input. For base excitation,
isolation begins beyond roughly r=sqrt(2), although damping and multiple
structural modes complicate the real aircraft. This makes balancing
motors and propellers, avoiding structural resonance, and filtering gyro
data part of control-system design rather than mere mechanical
housekeeping.

![](media/image3.png){width="3.15in" height="1.9466294838145233in"}

Fig. 3. Single-mode vibration transmissibility. The dangerous region is
the neighborhood of r=1, where a frame/IMU mode can amplify rotor
excitation.

# XI. SAMPLED-DATA ATTITUDE CONTROL

The YMFC-32 public project and the internal source material both use a
fixed 250 Hz loop. Brokking states that each loop is 4000 microseconds
and warns that exceeding this time disturbs the angle calculations and
can destabilize the vehicle \[3\]. The STM32F103C8 product documentation
specifies a 72 MHz Cortex-M3 CPU \[5\]. Thus the absolute clock budget
is 72e6/250 = 288,000 cycles per control interval, although the usable
budget is lower because interrupts and peripheral work also consume
time.

S = f_L/f_n (28)

The number S is the ratio of the digital sampling frequency to the
closed-loop bandwidth. It is more informative than saying only that
\'250 Hz is fast.\' If f_n=20 Hz, S=12.5; if f_n=50 Hz, S=5. The project
GDA treats S greater than roughly 10 as a comfortable initial design
target and S near 5 as marginal. These are engineering heuristics, not
universal stability theorems.

![](media/image4.png){width="3.15in" height="1.9466294838145233in"}

Fig. 4. Sampling ratio for the fixed 250 Hz control loop. The bandwidth
determines whether 250 Hz is generous or marginal.

## A. Cascaded Controller

The original architecture is physically sensible: the outer angle loop
asks for an angular rate; the inner rate loop asks the motors for
torque. In reduced form,

angle command -\> angle P/PI -\> rate command -\> rate PID -\> motor
mixer (29)

For a practical upgrade on the STM32F103, the most valuable changes are
not necessarily exotic control laws. A rate PID with derivative on
measurement, anti-windup, feedforward, explicit saturation handling, and
payload/inertia gain scheduling attacks the dominant nonidealities
directly. A compact quaternion complementary observer, such as the
nonlinear complementary-filter family of Mahony et al. \[10\], is also a
natural step beyond a basic Euler-angle complementary filter when
computational budget permits.

tau_c = K_P e_omega + K_I integral(e_omega dt) - K_D omega_dot + tau_ff
(30)

# XII. COMPUTE MODEL

The compute model should count processor cycles, not simply
\'instructions,\' because Cortex-M3 instructions do not all take one
cycle. Let C_loop be measured CPU cycles consumed by one control
iteration.

U_c = C_loop f_L / f_CPU (31)

With f_CPU=72 MHz and f_L=250 Hz, 288,000 cycles are available per
interval. The project\'s provisional 50,000-cycle surrogate corresponds
to U_c about 17%, but this number remains an estimate until measured on
the target firmware. Adding GPS, barometer, magnetometer, filtering,
telemetry, or more advanced estimation changes the cycle budget and
sometimes the worst-case execution path.

# XIII. ALLOWABLE PAYLOAD AS A MULTIPHYSICS CONSTRAINT

The earlier thrust-only payload calculation produced about 0.415 kg for
the 250 g sandbox when Pi_TW,min=1.5. Public YMFC-32 evidence shows why
that cannot be treated as an aircraft rating. Brokking recommends a
maximum payload of 200 g because heavier loads increase propeller
drag/stall, vibration, \'jello,\' and unstable flight behavior \[3\]. A
separate published build weighed 1.04 kg, had a recommended takeoff
weight of 1.20 kg, and therefore had only about 0.16 kg recommended mass
margin in that configuration \[4\].

m_p,allow = min(m_p,TW, m_p,end, m_p,vib, m_p,current, m_p,thermal,
m_p,structure) (32)

Equation (32) is the right engineering definition. The maximum useful
payload is whichever subsystem says \'no\' first. GDA is valuable here
because each limit can be written as a dimensionless margin, compared on
equal footing, and then replaced by calibrated data as testing proceeds.

# XIV. VALIDATION AGAINST PUBLIC YMFC-32 DATA

The strongest public quantitative reference found is Brokking\'s Hubsan
H109s-frame YMFC-32 build: 1.040 kg including batteries, 9.4x4.3 inch
propellers, 370 mm motor-to-motor spacing, recommended takeoff weight
1.20 kg, and 23 min flight time with two 2200 mAh 3S LiPo batteries
\[4\]. Another official page reports 15-20 min on Brokking\'s autonomous
test quadcopters using a 3800 mAh 3S pack \[2\].

For the 1.04 kg build, two 2.2 Ah 3S packs contain 48.84 Wh when
evaluated at the usual 11.1 V nominal pack voltage. Dividing nominal
energy by 23 min gives an energy-equivalent average power of 127.4 W.
Using the reduced momentum model with D=0.2388 m and the aggregate
factor FM\*eta_m=0.55\*0.70=0.385 gives 127.8 W. The difference is about
0.29%.

![](media/image5.png){width="3.15in" height="2.1460367454068243in"}

Fig. 5. Aggregate power-scale validation using the published 1.04 kg /
9.4-inch / 23-min build. The close agreement supports the scaling level,
not independent values of FM, motor efficiency, or usable battery
fraction.

The numerical closeness should not be overinterpreted. The battery
comparison uses nominal energy; actual delivered capacity and average
voltage were not published. Likewise, FM and eta_m were not measured
independently. The defensible conclusion is therefore that the
reduced-order power scale is consistent with the published flight point,
while the individual efficiency parameters remain unidentifiable from
that one datum.

  ------------------------------------------------------------------------
  **Quantity**           **Evidence status**      **Conclusion**
  ---------------------- ------------------------ ------------------------
  250 Hz loop            Direct                   Public Q&A states fixed
                                                  4000 us loop \[3\].

  72 MHz CPU             Direct                   ST specifies STM32F103C8
                                                  up to 72 MHz \[5\].

  MPU-6050 capability    Direct                   TDK datasheet supports
                                                  kHz-class sampling
                                                  \[6\].

  Hover power scaling    Aggregate                Published build is
                                                  consistent with
                                                  momentum-model power
                                                  scale.

  Payload \<=200 g       Direct recommendation    Brokking cites
                                                  vibration/stall
                                                  instability \[3\].

  C_T=0.192              Not validated for        Must be fitted from
                         published props          thrust/RPM data.

  C_D A=0.02 m\^2        Assumed                  Needs drag
                                                  identification.

  Altitude ceiling       Model only               No reliable public
                                                  YMFC-32 max-altitude
                                                  test found.

  f_n=20 Hz              Assumed/target           Needs closed-loop system
                                                  identification.
  ------------------------------------------------------------------------

# XV. NORMALIZED SYSTEM-LEVEL DESIGN EQUATIONS

The most useful design form is the normalized one. Choose a reference
aircraft or test point (subscript 0) and define hats as ratios, for
example m_hat=m/m_0. Then the model exposes what changes matter without
carrying units through every trade study.

T_hat = C_T_hat rho_hat n_hat\^2 D_hat\^4 (33)

P_hover_hat = m_hat\^(3/2) D_hat\^(-1) rho_hat\^(-1/2) (FM
eta_m)\_hat\^(-1) (34)

t_hat = E_hat m_hat\^(-3/2) D_hat rho_hat\^(1/2) (FM eta_m)\_hat (35)

F_D_hat = rho_hat V_hat\^2 (C_D A)\_hat (36)

These equations do not eliminate calibration. They tell us exactly what
must be calibrated. If a diameter sweep causes Reynolds number, tip Mach
number, motor current, or blade stall to change substantially, then C_T
and efficiency cannot be held fixed. That is the meaning of dynamic
similarity: matching geometry alone is insufficient \[7\].

# XVI. MATLAB AND PYTHON DIGITAL DESIGN IMPLEMENTATIONS

The accompanying MATLAB and Python implementations use the same
reduced-order design kernel. Both evaluate total mass and inertia,
maximum thrust, hover rotor speed, thrust-to-weight ratio, disk loading,
induced velocity, hover power, drag, endurance, thrust and energy
ceilings, sample ratio, actuator time-scale ratio, structural/vibration
ratios, compute utilization, and competing payload constraints.

  -----------------------------------------------------------------------
  **Module**                **Purpose**            **Primary outputs**
  ------------------------- ---------------------- ----------------------
  Core evaluation           One design point       Tmax, Phover, t, h,
                                                   payload margins

  Payload sweep             Mission trade space    Endurance, ceilings,
                                                   T/W, headroom

  Diameter sweep            GDA sensitivity        Tmax, endurance, Re,
                                                   Ma_tip

  Vibration model           IMU/frame screening    f_1P, f_BPF, T_v

  Control/compute           Timing screening       S, Pi_m, Pi_delay, U_c

  Validation check          Published build        Power-scale
                                                   consistency
  -----------------------------------------------------------------------

The codes deliberately label C_T, n_max, motor time constant, structural
frequency/damping, drag area, closed-loop bandwidth, and cycles per loop
as calibration quantities. This is a feature, not a weakness: a digital
model is most useful when it advertises uncertainty instead of hiding
it.

# XVII. EXPERIMENTAL CALIBRATION PLAN

Four experiments would convert the present reduced model from a
physically constrained estimator into a calibrated YMFC-32 digital
design model:

**•** Thrust stand: measure thrust, electrical power, and RPM over
command; fit C_T(n), C_P(n), and the aggregate propulsion efficiency.

**•** Motor step test: command small and large ESC steps while measuring
RPM; identify K_m and tau_m.

**•** Frame/IMU modal test: excite the airframe and estimate dominant
structural frequencies and damping; compare them with 1P and
blade-passing frequencies.

**•** Closed-loop identification: apply safe small-amplitude roll/pitch
steps or chirps; estimate f_n, damping, delay, and control saturation
margins.

# XVIII. DESIGN INSIGHTS FOR POSTGRADUATE READERS

The reduced model supports several conclusions that are easy to remember
because they are attached to physical pictures rather than isolated
formulas. First, thrust is fundamentally a momentum-flux problem, so
disk area and rotor speed dominate. Second, hover endurance is an
energy-over-power problem, and power grows approximately as m\^(3/2);
payload therefore hurts endurance faster than linearly. Third, control
quality is a time-scale problem: the loop frequency must be judged
relative to the aircraft bandwidth, motor lag, delay, and structural
resonances. Fourth, the IMU is part of the mechanics because it sits on
a vibrating structure. Fifth, maximum payload and maximum altitude are
not single-formula properties; they are minima over competing
constraints.

The deepest lesson of GDA is therefore not a particular exponent. It is
a disciplined way of asking: what are the independent ratios that decide
the regime? Once those ratios are identified, a complicated aircraft
becomes a smaller set of physical questions that can be tested one by
one.

# XIX. LIMITATIONS

The present model is intentionally reduced order. It neglects detailed
blade-element aerodynamics, rotor-rotor interaction, ground effect,
voltage sag and battery internal resistance, ESC current limits,
temperature-dependent motor constants, flexible multi-mode airframe
dynamics, nonlinear inflow in aggressive forward flight, and full 6-DOF
coupled simulation. Those effects should be added only when the simpler
model fails a measured comparison or when a design decision depends on
them. The reported altitude outputs are physical-model quantities only
and should not be interpreted as operational authorization.

# XX. CONCLUSION

A coherent YMFC-32 design model emerges when GDA is used as the
organizing language and mechanics supplies the missing functions. The
rotor groups determine thrust and similarity; momentum theory connects
weight and disk area to power; battery energy determines endurance;
rigid-body inertia and motor lag determine controllability; structural
dynamics determine what the IMU actually senses; sampled-data ratios
determine whether a 250 Hz controller is fast enough; and compute
utilization determines whether the algorithm can be executed in time.
Public YMFC-32 data support the architecture and provide a useful
aggregate power-validation point, while also showing that vibration and
propeller loading can impose payload limits well before a static thrust
calculation does. The MATLAB and Python models produced in this project
provide a practical path from these equations to calibrated parametric
design.

# NOMENCLATURE

  -----------------------------------------------------------------------
  **Symbol**                         **Meaning**
  ---------------------------------- ------------------------------------
  m, m_b, m_p                        total, battery, and payload mass

  D, n, omega                        propeller diameter, rev/s, and rad/s
                                     rotor speed

  C_T, C_P                           thrust and power coefficients

  FM, eta_m                          rotor figure of merit and motor/ESC
                                     efficiency

  rho, mu, c                         air density, dynamic viscosity,
                                     speed of sound

  T, F_D, P                          thrust, drag, and power

  I, tau                             rigid-body inertia and applied
                                     torque

  f_L, f_n                           control-loop frequency and
                                     closed-loop bandwidth

  tau_m, tau_d                       motor time constant and effective
                                     delay

  f_s, zeta_s                        structural natural frequency and
                                     damping ratio

  U_c                                normalized processor utilization
  -----------------------------------------------------------------------

# PROJECT ARTIFACTS

The report synthesizes the uploaded project documents
ymfc32_drone_gda_summary.md, ymfc32_drone_design_gda.md,
ymfc32_gda_validate.py, ymfc32_design_gda_validate.py, and
gda_workflow_playbook_v4.md, together with the project MATLAB and Python
reduced-order implementations. These are working project artifacts
rather than independently published external references.

# REFERENCES

\[1\] J. Brokking, "Project YMFC-32 - The STM32 quadcopter,"
Brokking.net.
[[https://www.brokking.net/ymfc-32_main.html]{.underline}](https://www.brokking.net/ymfc-32_main.html)

\[2\] J. Brokking, "Project YMFC-32 autonomous," Brokking.net.
[[https://www.brokking.net/ymfc-32_auto_main.html]{.underline}](https://www.brokking.net/ymfc-32_auto_main.html)

\[3\] J. Brokking, "Project YMFC-32 autonomous - Q & A," Brokking.net.
[[https://www.brokking.net/ymfc-32_auto_qanda.html]{.underline}](https://www.brokking.net/ymfc-32_auto_qanda.html)

\[4\] J. Brokking, "Building the YMFC-32 in a Hubsan H109s X4 Pro
frame," Brokking.net.
[[https://www.brokking.net/bain_YMFC-32_Hubsan_frame.html]{.underline}](https://www.brokking.net/bain_YMFC-32_Hubsan_frame.html)

\[5\] STMicroelectronics, "STM32F103C8 - Arm Cortex-M3 MCU, 72 MHz,"
product page and datasheet.
[[https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8]{.underline}](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8)

\[6\] TDK InvenSense, "MPU-6000/MPU-6050 Product Specification," Rev.
3.4.
[[https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf]{.underline}](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)

\[7\] G. H. McKinley, "The Buckingham Pi Theorem in Dimensional
Analysis," MIT OpenCourseWare, 2.25 Advanced Fluid Mechanics.
[[https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_the_buckingham/]{.underline}](https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_the_buckingham/)

\[8\] NASA, "Momentum Theory," in rotorcraft aerodynamics material, NASA
Technical Reports Server.
[[https://ntrs.nasa.gov/api/citations/19790013868/downloads/19790013868.pdf]{.underline}](https://ntrs.nasa.gov/api/citations/19790013868/downloads/19790013868.pdf)

\[9\] K. J. Astrom and R. M. Murray, Feedback Systems: An Introduction
for Scientists and Engineers, Princeton Univ. Press.
[[https://www.cds.caltech.edu/\~murray/FBS/First_Edition.html]{.underline}](https://www.cds.caltech.edu/~murray/FBS/First_Edition.html)

\[10\] R. Mahony, T. Hamel, and J.-M. Pflimlin, "Nonlinear Complementary
Filters on the Special Orthogonal Group," IEEE Trans. Autom. Control,
vol. 53, no. 5, pp. 1203-1218, 2008, doi:10.1109/TAC.2008.923738.
[[https://doi.org/10.1109/TAC.2008.923738]{.underline}](https://doi.org/10.1109/TAC.2008.923738)

\[11\] R. W. Beard and T. W. McLain, Small Unmanned Aircraft: Theory and
Practice, Princeton Univ. Press, 2012.
[[https://www.mathworks.com/academia/books/small-unmanned-aircraft-beard.html]{.underline}](https://www.mathworks.com/academia/books/small-unmanned-aircraft-beard.html)

\[12\] J. Brokking, "YMFC-32 downloads - schematic and software
package," Brokking.net.
[[https://www.brokking.net/ymfc-32_downloads.html]{.underline}](https://www.brokking.net/ymfc-32_downloads.html)

\[13\] AI Agent for Generlized Dimensional Analysis:

<https://chatgpt.com/g/g-whgfvDwJW-dimensional-analysis-guru>
