from sympy import symbols, solve
import numpy as np
import matplotlib.pyplot as plt

v0 = 7.90897; vt = 25.0; p0 = -340.689; pt = 0
ais = []; bis = []; js = []
tms = np.linspace(15.0, 50.0, 70)
for tm in tms:
    ai, bi = symbols('ai bi')
    results = solve((1/2*ai*tm**2 + bi*tm + v0 - vt,
        1/6*ai*tm**3 + 1/2*bi*tm**2 + v0*tm + p0 - pt),
        (ai,bi), dict=True)
    ai = float(results[0][ai])
    bi = float(results[0][bi])
    j = ((ai*tm+bi)**3-bi**3)/ai
    ais.append(ai)
    bis.append(bi)
    js.append(j)

plt.plot(tms, np.array(js))
plt.show()