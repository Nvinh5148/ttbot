#!/usr/bin/env python3
import sys
import csv
import matplotlib.pyplot as plt

if len(sys.argv) > 1:
    path_file = sys.argv[1]
else:
<<<<<<< HEAD
<<<<<<< HEAD
    path_file = "path_uturn.csv"
=======
    path_file = "path_u_to_S.csv"
>>>>>>> c905fff ( available mpc)
=======
    path_file = "path_right.csv"
>>>>>>> 9a9fa327999ee5683041f4e1f2666fe709c40b3f

xs = []
ys = []

with open(path_file) as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        xs.append(float(row[0]))
        ys.append(float(row[1]))

# ==== ĐỔI TRỤC (X <-> Y) ====
<<<<<<< HEAD
plt.plot(ys, xs, "-o")

# ==== Nếu muốn lật Y lại cho cùng chiều ROS, bật dòng dưới ====
# plt.gca().invert_yaxis()
=======
plt.plot(ys, xs, "o")

# ==== Nếu muốn lật Y lại cho cùng chiều ROS, bật dòng dưới ====
#plt.gca().invert_yaxis()
>>>>>>> c905fff ( available mpc)

plt.xlabel("Y (m)")
plt.ylabel("X (m)")
plt.axis("equal")
plt.grid(True)
plt.title(path_file)
<<<<<<< HEAD
plt.show()
=======
plt.show()
>>>>>>> c905fff ( available mpc)
