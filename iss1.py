from matplotlib import pyplot as plt
import math
import json

# init
hmaks = 15  # wysokosc zbiornika
n = 100  # ilosc iteracji badania
dt = 0.1  # probkowanie co 0.01s, to samo co Tp
hzad = 16  # wysokosc slupa zadana
h0 = 1  # wysokosc slupa poczatkowa
A = 8  # powierzchnia podstawy
betha = 0.1  # wspolczynnik wyplywu: Q0(t) = betha * sqrt(h(t))
Qd = 0  # poczatkowe natezenie doplywu
K_p = 3.0
K_i = 1.0
K_d = 0.3

with open('config.json') as json_file:
    data = json.load(json_file)
    hmaks = float(data['hmax'])
    n = int(data['n'])
    dt = float(data['dt'])
    hzad = float(data['hzad'])
    h0 = float(data['h0'])
    A = float(data['A'])
    betha = float(data['betha'])
    Qd = float(data['Qd0'])
    K_p = float(data['kp'])
    K_i = float(data['ki'])
    K_d = float(data['kd'])


def PID(Kp, Ki, Kd, MV_bar=0):
    # initialize stored data
    e_prev = 0
    t_prev = -100
    I = 0

    # initial control
    MV = MV_bar

    while True:
        # yield MV, wait for new t, PV, SP
        t, PV, SP = yield MV

        # PID calculations
        e = SP - PV

        P = Kp * e
        I = I + Ki * e * (t - t_prev)
        D = Kd * (e - e_prev) / (t - t_prev)

        # zabezpieczenie przed zjawiskiem wind-up'u
        if I > 2:
            I = 2

        MV = MV_bar + P + I + D
        if MV < 0:
            MV = 0
        elif MV > 150.5:
            MV = 150.5
        # update stored data for next iteration
        e_prev = e
        t_prev = t


# PROCES I


h_pid = [0 for i in range(n + 1)]
h_pid[0] = h0
time_pid = [i * dt for i in range(n + 1)]

controller = PID(K_p, K_i, K_d)  # create pid control
controller.send(None)  # initialize

for i in range(n):
    # h[i+1] = controller.send([time[i], ((1/A)*(-betha * math.sqrt(h[i]) + Qd)*dt + h[i]), 5])
    print(h_pid[i])
    Qd = controller.send([time_pid[i], ((1 / A) * (-betha * math.sqrt(h_pid[i]) + Qd) * dt + h_pid[i]), hzad])
    h_pid[i + 1] = (1 / A) * (-betha * math.sqrt(h_pid[i]) + Qd) * dt + h_pid[i]
    if h_pid[i + 1] < 0:
        h_pid[i + 1] = 0.0

# logika rozmyta
import skfuzzy
from skfuzzy import control as ctrl
import numpy as np

error = ctrl.Antecedent(np.arange(-2, 2, 0.01), 'error')
error_delta = ctrl.Antecedent(np.arange(-2, 2, 0.01), 'error_delta')
output = ctrl.Consequent(np.arange(0, 2, 0.01), 'output')

error['DU'] = skfuzzy.trapmf(error.universe, [-2, -2, -2, -1])
error['ŚU'] = skfuzzy.trimf(error.universe, [-1.8, -1, -0.2])
error['MU'] = skfuzzy.trimf(error.universe, [-0.4, -0.2, 0])
error['Z'] = skfuzzy.trimf(error.universe, [-0.1, 0, 0.1])
error['MD'] = skfuzzy.trimf(error.universe, [0, 0.2, 0.4])
error['ŚD'] = skfuzzy.trimf(error.universe, [0.2, 1, 1.8])
error['DD'] = skfuzzy.trapmf(error.universe, [1, 2, 2, 2])

error_delta['DU'] = skfuzzy.trapmf(error_delta.universe, [-2, -2, -2, -1])
error_delta['ŚU'] = skfuzzy.trimf(error_delta.universe, [-1.8, -1, -0.2])
error_delta['MU'] = skfuzzy.trimf(error_delta.universe, [-0.4, -0.2, 0])
error_delta['Z'] = skfuzzy.trimf(error_delta.universe, [-0.1, 0, 0.1])
error_delta['MD'] = skfuzzy.trimf(error_delta.universe, [0, 0.2, 0.4])
error_delta['ŚD'] = skfuzzy.trimf(error_delta.universe, [0.2, 1, 1.8])
error_delta['DD'] = skfuzzy.trapmf(error_delta.universe, [1, 2, 2, 2])

output['Z'] = skfuzzy.trimf(output.universe, [0, 0.2, 0.4])
output['MD'] = skfuzzy.trimf(output.universe, [0.2, 0.4, 0.6])
output['ŚD'] = skfuzzy.trimf(output.universe, [0.4, 0.6, 0.8])
output['DD'] = skfuzzy.trimf(output.universe, [0.6, 0.8, 1.0])
output['BDD'] = skfuzzy.trapmf(output.universe, [0.8, 1.2, 2, 2])

rule0 = ctrl.Rule(antecedent=(
        (error['DU'] & error_delta['DU']) | (error['DU'] & error_delta['ŚU']) | (error['DU'] & error_delta['MU']) |
        (error['DU'] & error_delta['Z']) | (error['DU'] & error_delta['MD']) | (error['DU'] & error_delta['ŚD']) |
        (error['DU'] & error_delta['DD']) |
        (error['ŚU'] & error_delta['DU']) | (error['ŚU'] & error_delta['ŚU']) | (error['ŚU'] & error_delta['MU']) |
        (error['ŚU'] & error_delta['Z']) | (error['ŚU'] & error_delta['MD']) | (error['ŚU'] & error_delta['ŚD']) |
        (error['MU'] & error_delta['DU']) | (error['MU'] & error_delta['ŚU']) | (error['MU'] & error_delta['MU']) |
        (error['MU'] & error_delta['Z']) | (error['MU'] & error_delta['MD']) |
        (error['Z'] & error_delta['DU']) | (error['Z'] & error_delta['ŚU']) | (error['Z'] & error_delta['MU']) |
        (error['Z'] & error_delta['Z']) |
        (error['MD'] & error_delta['DU']) | (error['MD'] & error_delta['ŚU']) | (error['MD'] & error_delta['MU']) |
        (error['ŚD'] & error_delta['DU']) | (error['ŚD'] & error_delta['ŚU']) |
        (error['DD'] & error_delta['DU'])
), consequent=output['Z'], label='rule Z')

rule1 = ctrl.Rule(antecedent=(
        (error['ŚU'] & error_delta['DD']) |
        (error['MU'] & error_delta['DU']) |
        (error['Z'] & error_delta['MD']) |
        (error['MD'] & error_delta['Z']) |
        (error['ŚD'] & error_delta['MU']) |
        (error['DD'] & error_delta['ŚU'])
), consequent=output['MD'], label='rule MD')

rule2 = ctrl.Rule(antecedent=(
        (error['MU'] & error_delta['DD']) |
        (error['Z'] & error_delta['ŚD']) |
        (error['MD'] & error_delta['MD']) |
        (error['ŚD'] & error_delta['Z']) |
        (error['DD'] & error_delta['MU'])
), consequent=output['ŚD'], label='rule ŚD')

rule3 = ctrl.Rule(antecedent=(
        (error['Z'] & error_delta['DD']) |
        (error['MD'] & error_delta['ŚD']) |
        (error['ŚD'] & error_delta['MD']) |
        (error['DD'] & error_delta['Z'])
), consequent=output['DD'], label='rule DD')

rule4 = ctrl.Rule(antecedent=(
        (error['MD'] & error_delta['DD']) |
        (error['ŚD'] & error_delta['ŚD']) | (error['ŚD'] & error_delta['DD']) |
        (error['DD'] & error_delta['MD']) | (error['DD'] & error_delta['ŚD']) | (error['DD'] & error_delta['DD'])
), consequent=output['BDD'], label='rule BDD')

system = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4])
simulation = ctrl.ControlSystemSimulation(system)

h_fuzzy = [0 for i in range(n + 1)]
h_fuzzy[0] = h0
time_fuzzy = [i * dt for i in range(n + 1)]

e = [0 for i in range(n)]
e_d = [0 for i in range(n + 1)]
q = [0 for i in range(n)]

for i in range(n):
    simulation.input['error'] = hzad - h_fuzzy[i]
    e[i] = hzad - h_fuzzy[i]
    if (i > 0):
        simulation.input['error_delta'] = (hzad - h_fuzzy[i]) - (hzad - h_fuzzy[i - 1])
        e_d[i]= (hzad - h_fuzzy[i]) - (hzad - h_fuzzy[i - 1])
    else:
        simulation.input['error_delta'] = (hzad - h_fuzzy[i])
        e_d[i] = (hzad - h_fuzzy[i])

    simulation.compute()
    Qd = simulation.output['output']
    q[i] = Qd
    h_fuzzy[i + 1] = (1 / A) * (-betha * math.sqrt(h_fuzzy[i]) + Qd) * dt + h_fuzzy[i]
    if h_fuzzy[i + 1] < 0:
        h_fuzzy[i + 1] = 0.0

fig = plt.figure()
plt.plot(time_fuzzy, h_fuzzy)
plt.xlabel('czas[s]')
plt.ylabel('Wysokość[m]')
plt.title('Wysokość słupa wody-fuzzy')
fig.show()

fig = plt.figure()
plt.plot(time_fuzzy[0:n], e)
plt.title('error-fuzzy')
fig.show()

fig = plt.figure()
plt.plot(time_fuzzy, e_d)
plt.title('error_delta-fuzzy')
fig.show()

fig = plt.figure()
plt.plot(time_fuzzy[0:n], q)
plt.title('Qd-fuzzy')
fig.show()

fig = plt.figure()
plt.plot(time_pid, h_pid)
plt.xlabel('czas[s]')
plt.ylabel('Wysokość[m]')
plt.title('Wysokość słupa wody-pid')
fig.show()

plt.show()

wholeData = []

with open("data_fuzzy.json", "w") as write_file:
    for i in range(0, n):
        data = ({
            "t": time_fuzzy[i],
            "y": h_fuzzy[i]
        })

        wholeData.append(data)
    json.dump(wholeData, write_file)
    write_file.close()
wholeData = []

with open("data_pid.json", "w") as write_file:
    for i in range(0, n):
        data = ({
            "t": time_pid[i],
            "y": h_pid[i]
        })

        wholeData.append(data)
    json.dump(wholeData, write_file)
    write_file.close()

