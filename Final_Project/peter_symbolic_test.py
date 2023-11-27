import sympy as sp

# Define symbolic variables
theta1, theta2 = sp.symbols('theta1 theta2')
thetad1, thetad2 = sp.symbols('thetad1 thetad2')
thetadd1, thetadd2 = sp.symbols('thetadd1 thetadd2')
tau1, tau2 = sp.symbols('tau1 tau2')

m1, m2 = sp.symbols('m1 m2')
l1, l2 = sp.symbols('l1 l2')

# Define symbolic equations
# Example equations (replace these with your actual equations)
m11 = m1 * l1**2 + m2 * (l1**2 + 2 * l1 * l2 * sp.cos(theta2) + l2**2)
m12 = m21 = m2 * (l2**2 + l1 * l2 * sp.cos(theta2))
m22 = m2 * l2**2

c1 = -m2*l1*l2*sp.sin(theta2)*(2*thetad1*thetad2 + thetad2**2)
c2 = m2*l1*l2*thetad1**2*sp.sin(theta2)

M = sp.Matrix([[m11, m12], [m21, m22]])  # Inertia matrix M(theta)
C = sp.Matrix([[c1], [c2]])  # Coriolis and centrifugal matrix C(theta, thetad)
# G = sp.Matrix([[g1], [g2]])  # Gravity vector G(theta)

# Assuming tau = M * thetadd + C * thetad + G
thetadd = sp.Matrix([[thetadd1], [thetadd2]])
tau = sp.Matrix([[tau1], [tau2]])

# Solve for thetadd
# thetadd_solution = sp.solve(M * thetadd + C * sp.Matrix([[thetad1], [thetad2]]) + G - tau, thetadd)
thetadd_solution = sp.solve(M * thetadd + C - tau, thetadd)

# Print the solution for thetadd
print("Solution for thetadd:")
print(thetadd_solution)
