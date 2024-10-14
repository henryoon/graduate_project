import numpy as np

diagonal_matrix = np.diag(np.arange(1, 7))
print(diagonal_matrix)

res = np.diag(diagonal_matrix)[3:6]
print(res)
