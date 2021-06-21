from flask import Flask, render_template, request, redirect, send_file
from matplotlib import pyplot as plt
import math
import json
import skfuzzy
from skfuzzy import control as ctrl
import numpy as np

import os

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


h = [0 for i in range(n + 1)]
h[0] = h0
time = [i * dt for i in range(n + 1)]

controller = PID(K_p, K_i, K_d)  # create pid control
controller.send(None)  # initialize

for i in range(n):
    # h[i+1] = controller.send([time[i], ((1/A)*(-betha * math.sqrt(h[i]) + Qd)*dt + h[i]), 5])
    print(h[i])
    Qd = controller.send([time[i], ((1 / A) * (-betha * math.sqrt(h[i]) + Qd) * dt + h[i]), hzad])
    h[i + 1] = (1 / A) * (-betha * math.sqrt(h[i]) + Qd) * dt + h[i]
    if h[i + 1] < 0:
        h[i + 1] = 0.0

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

# error.view()
# error_delta.view()
# output.view()
# plt.show()


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

h = [0 for i in range(n + 1)]
h[0] = h0
time = [i * dt for i in range(n + 1)]

e = [0 for i in range(n)]
e_d = [0 for i in range(n + 1)]
q = [0 for i in range(n)]

for i in range(n):
    simulation.input['error'] = hzad - h[i]
    e[i] = hzad - h[i]
    if (i > 0):
        simulation.input['error_delta'] = (hzad - h[i]) - (hzad - h[i - 1])
        e_d[i] = (hzad - h[i]) - (hzad - h[i - 1])
    else:
        simulation.input['error_delta'] = (hzad - h[i])
        e_d[i] = (hzad - h[i])

    simulation.compute()
    Qd = simulation.output['output']
    q[i] = Qd
    h[i + 1] = (1 / A) * (-betha * math.sqrt(h[i]) + Qd) * dt + h[i]
    if h[i + 1] < 0:
        h[i + 1] = 0.0

app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
imSavePath = "./"


@app.route('/')
def hello():
    return 'Hello, World!'


@app.route('/fuzzyPlots', methods=["POST", "GET"])
def fuzzyPlots():
    global time, h, e, e_d, q
    xlabel = time
    values = h
    return render_template("public/iss_page.html", xlabel=xlabel, values=values, values_e=e, values_e_d=e_d, values_q=q)
@app.route('/pidPlots', methods=["POST", "GET"])
def pidPlots():


if __name__ == '__main__':
    app.run(debug=True)  # run our app
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
