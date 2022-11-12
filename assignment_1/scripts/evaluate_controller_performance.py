import argparse
import numpy as np
import matplotlib.pyplot as plt

from sdc_course.utils.utility import load_waypoints_from_txt, waypoint_distance


def plot_trajectory_comparison(reference_trajectory, tracked_trajectory):
    reference_trajectory_np = np.array(reference_trajectory)
    tracked_trajectory_np = np.array(tracked_trajectory)

    fig = plt.figure()
    ax1 = plt.subplot(121)
    plt.title("Trajectory")
    plt.plot(reference_trajectory_np[:, 0], reference_trajectory_np[:, 1], "-r", label="Reference")
    plt.plot(tracked_trajectory_np[:, 0], tracked_trajectory_np[:, 1], "--g", label="Tracked")
    ax1.legend()

    ax2 = plt.subplot(122)
    plt.title("Velocity Profile")
    plt.plot(reference_trajectory_np[:, 2], "-r", label="Reference")
    plt.plot(tracked_trajectory_np[:, 2], "--g", label="Tracked")
    ax2.legend()
    plt.show()


def compute_tracked_distance(trajectory):
    tracked_distance = 0
    for i in range(1, len(trajectory)):
        tracked_distance += waypoint_distance(trajectory[i], trajectory[i - 1])
    return tracked_distance


def evaluate_trajectory_simple_check(reference_trajectory, tracked_trajectory):
    L_reference = compute_tracked_distance(reference_trajectory)
    L_tracked = compute_tracked_distance(tracked_trajectory)
    L_ratio = L_tracked / L_reference
    print("Length of Tracked/Reference trajectory = {:.3f}/{:.3f}".format(L_tracked, L_reference))
    print("Ratio = {} ".format(L_ratio))

    if L_ratio > 0.95 and L_ratio < 1.05:
        print("Tracking Performance: Good")
    elif L_ratio > 0.90 and L_ratio < 0.95:
        print("Tracking Performance: Okay")
    else:
        print("Tracking Performance: Bad")


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="Evaluate Controller Performance.")
    argparser.add_argument(
        "-r",
        "--ref-traj",
        default="results/recorded_trajectory.txt",
        dest="ref_traj",
        help="Path to reference trajectory file.",
    )
    argparser.add_argument(
        "-t",
        "--tracked-traj",
        default="results/tracked_trajectory.txt",
        dest="tracked_traj",
        help="Path to tracked trajectory file.",
    )
    args = argparser.parse_args()

    # Load two trajectories
    reference_trajectory = load_waypoints_from_txt(args.ref_traj)
    tracked_trajectory = load_waypoints_from_txt(args.tracked_traj)

    # plot comparisons
    plot_trajectory_comparison(reference_trajectory, tracked_trajectory)

    # evaluate trajectory
    evaluate_trajectory_simple_check(reference_trajectory, tracked_trajectory)
