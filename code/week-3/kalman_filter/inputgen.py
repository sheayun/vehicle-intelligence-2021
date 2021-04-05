import numpy as np

dt = 0.1
N = 100
sigW = 10
sigV = 10

W = np.random.normal(0, sigW, N)
V = np.random.normal(0, sigV, N)

Posp = 0
Z = []
P = []
S = []

for w, v in zip(W, V):
    Velp = 80 + w
    z = Posp + Velp * dt + v
    Posp = z - v
    Z.append(z)
    P.append(Posp)
    S.append(Velp)

print("measurements = [")
for z in Z:
    print("\t%e," % z)
print("]")
print("")
print("true_positions = [")
for p in P:
    print("\t%e," % p)
print("]")
print("")
print("true_velocity = [")
for s in S:
    print("\t%e," % s)
print("]")
