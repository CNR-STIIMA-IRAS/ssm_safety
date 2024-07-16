import sympy as sym

vr=sym.Symbol('vr')
vh=sym.Symbol('vh')
Tr=sym.Symbol('Tr')
a=sym.Symbol('a')
d=-a

print("Positive velocity")
# v=vr+d*Tbreak =0
Tbreak=-vr/d
C=sym.Symbol('C')
D=sym.Symbol('D')
Sh=vh*(Tr+Tbreak)
Sr=(vr*Tr) +vr*Tbreak + 1/2*d*(Tbreak**2)
distance=D+Sh-Sr
print(f"equation: {distance}-{C}>0")
solutions=sym.simplify(sym.solveset(distance-C,vr))

for s in solutions:
  print(f"solution: {s}")



print("Negative velocity")
Tbreak=-vr/a
C=sym.Symbol('C')
D=sym.Symbol('D')
Sh=vh*(Tr+Tbreak)
Sr=(vr*Tr) +vr*Tbreak +1/2*a*(Tbreak**2)
distance=D+Sh-Sr

print(f"equation: {distance}-{C}>0")
solutions=sym.simplify(sym.solveset(distance-C,vr))

for s in solutions:
  print(f"solution: {s}")

