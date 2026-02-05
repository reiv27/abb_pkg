#!/usr/bin/env python3
"""
Строит 3D-график точек из egm_points.csv (target и feedback) с помощью matplotlib.
Запуск: python3 plot_egm_3d.py [путь_к_файлу]
"""

import csv
import os
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def find_csv_path():
    """Путь к CSV: аргумент, папка скрипта или $HOME/egm_points.csv."""
    if len(sys.argv) > 1:
        return sys.argv[1]
    local = os.path.join(os.path.dirname(__file__), "egm_points.csv")
    if os.path.isfile(local):
        return local
    home = os.environ.get("HOME", "")
    if home:
        path = os.path.join(home, "egm_points.csv")
        if os.path.isfile(path):
            return path
    return local


def load_csv(path):
    """Читает CSV, возвращает списки target_xyz и feedback_xyz."""
    target_x, target_y, target_z = [], [], []
    feedback_x, feedback_y, feedback_z = [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            target_x.append(float(row["target_x"]))
            target_y.append(float(row["target_y"]))
            target_z.append(float(row["target_z"]))
            feedback_x.append(float(row["feedback_x"]))
            feedback_y.append(float(row["feedback_y"]))
            feedback_z.append(float(row["feedback_z"]))
    return (target_x, target_y, target_z), (feedback_x, feedback_y, feedback_z)


def main():
    path = find_csv_path()
    if not os.path.isfile(path):
        print(f"Файл не найден: {path}", file=sys.stderr)
        sys.exit(1)

    target_xyz, feedback_xyz = load_csv(path)
    tx, ty, tz = target_xyz
    fx, fy, fz = feedback_xyz

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(tx, ty, tz, c="blue", marker="o", s=50, label="Target", alpha=0.9)
    ax.scatter(fx, fy, fz, c="green", marker="^", s=50, label="Feedback", alpha=0.9)

    for i in range(len(tx)):
        ax.plot(
            [tx[i], fx[i]],
            [ty[i], fy[i]],
            [tz[i], fz[i]],
            "k-",
            alpha=0.3,
            linewidth=1,
        )

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.legend()
    ax.set_title("EGM points: target (blue) vs feedback (green)")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
