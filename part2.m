A = 8 * 10^4
alpha = 0.602
Vc = 5.0
Rl = 10.0^4

C = 1:0.1:100 %c from 1 to 100
Rinf = A * C.^-alpha 


% Problem 2

%plot(C,Rinf) %linear graph
loglog(C, Rinf) %logarithmic graph
xlabel("VOCs (ppm)")
ylabel("R(inf) (ohms)")
title("Steady State Resistance vs. VOC Concentration")

% Problem 3
Vout = Vc * Rl ./ (Rl + A * C.^(-alpha))
plot(C, Vout)
xlabel("VOCs (ppm)")
ylabel("Vout (V)")
title("Output Voltage vs. VOC Concentration ")

%Problem 4
tau = 22
t = linspace(1, 100)
u_response = 1 - exp(-t/tau)
i_response = 1/tau * exp(-t/tau)

plot(t, i_response)
xlabel("Time (s)")
ylabel("Rs (ohms)")
title("Rs vs. Time (Impulse)")

plot(t, u_response)
xlabel("Time (s)")
ylabel("Rs (ohms)")
title("Rs vs. Time (Step)")

u2_response = 30360 - 22769*u_response %part vi, instantaneous change in C
plot(t, u2_response)
xlabel("Time (s)")
ylabel("Rs (ohms)")
title("Rs vs. Time")

Vout_2 = Vc * Rl ./ (Rl + u2_response)
plot(t, Vout_2)
xlabel("Time (s)")
ylabel("Output Voltage (V)")
title("Output Voltage vs. Time")

% Problem 5
v_ref = 5.0
v_delta = v_ref / 2^8
samp_t = 0:0.1:100
samp_vout = Vc * Rl ./ (Rl + (30360 -22769*(1 - exp(-samp_t/tau))))
quantize = (floor(samp_vout/v_delta) + 0.5) * v_delta

t_cont = 0:0.001:100 %for approximating ct
cont_vout = Vc * Rl ./ (Rl + (30360 -22769*(1 - exp(-t_cont/tau))))


% non quantized, sampled at 0.1 Ts
plot(samp_t, samp_vout)
xlabel("Time (s)")
ylabel("Output Voltage (V)")
title("Output Voltage vs. Time")

% 8 bit
plot(samp_t, [quantize; samp_vout])
hold on
plot(t_cont, cont_vout)
hold off
xlabel("Time (s)")
ylabel("Output Voltage (V)")
title("Output Voltage vs. Time (8 bit)")

v_delta = v_ref / 2^10
quantize = (floor(samp_vout/v_delta) + 0.5) * v_delta

% 10 bit
plot(samp_t, [quantize; samp_vout])
hold on
plot(t_cont, cont_vout)
hold off
xlabel("Time (s)")
ylabel("Output Voltage (V)")
title("Output Voltage vs. Time (10 bit)")

v_delta = v_ref / 2^12
quantize = (floor(samp_vout/v_delta) + 0.5) * v_delta

% 12 bit
plot(samp_t, [quantize; samp_vout])
hold on
plot(t_cont, cont_vout)
hold off
xlabel("Time (s)")
ylabel("Output Voltage (V)")
title("Output Voltage vs. Time (12 bit)")

% part v

v_delta = v_ref / 2^8
quantize = (floor(Vout/v_delta) + 0.5) * v_delta
c_reconstructed = (Vc*Rl ./ (A*quantize) - (Rl/A)).^(-1/alpha)
err8 = abs(c_reconstructed-C) ./C * 100
plot(C, err8)
xlabel("VOCs (ppm)")
ylabel("Relative Error (%)")
title("VOC Reconstruction Error(8 bit)")

v_delta = v_ref / 2^10
quantize = (floor(Vout/v_delta) + 0.5) * v_delta
c_reconstructed = (Vc*Rl ./ (A*quantize) - (Rl/A)).^(-1/alpha)
err10 = abs(c_reconstructed-C) ./C * 100
plot(C, err10)
xlabel("VOCs (ppm)")
ylabel("Relative Error (%)")
title("VOC Reconstruction Error (10 bit)")

v_delta = v_ref / 2^12
quantize = (floor(Vout/v_delta) + 0.5) * v_delta
c_reconstructed = (Vc*Rl ./ (A*quantize) - (Rl/A)).^(-1/alpha)
err12 = abs(c_reconstructed-C) ./C * 100
plot(C, err12)
xlabel("VOCs (ppm)")
ylabel("Relative Error (%)")
title("VOC Reconstruction Error (12 bit)")

plot(C, [err8; err10; err12])
xlabel("VOCs (ppm)")
ylabel("Relative Error (%)")
title("VOC Reconstruction Error (all)")