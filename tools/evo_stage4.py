#!/bin/python
import os
import argparse
import copy
import json
from evo.core import sync, geometry
from evo.core.metrics import *
from evo.tools.file_interface import *
from evo.main_ape import *
from evo.main_rpe import *
from evo.tools import plot
# This is a small program to evaluate the error of a visual-inertial system
# Note : install the evo package first
#   https://michaelgrupp.github.io/evo/

def update_res(ape_res:Result, rpe_res:Result, outputjson:str):
    with open(outputjson, 'r') as file:
        data = json.load(file)
    res = {"APE":dict(ape_res.stats.items()), "RPE":dict(rpe_res.stats.items())}
    data.update(res)
    with open(outputjson, 'w') as file:
        json.dump(data, file, indent=4)



def align_trajectory_whole(trj_est, trj_gt):
    trj_gt_sync, trj_est_sync = sync.associate_trajectories(trj_gt, trj_est)
    trj_copy = copy.deepcopy(
        trj_est_sync
    )  # otherwise np arrays will be references and mess up stuff
    r_a, t_a, s = geometry.umeyama_alignment(
        trj_copy.positions_xyz.T, trj_gt_sync.positions_xyz.T, False
    )
    res = lie.se3(r_a, t_a)
    return res


def align_trajectory(trj_est, trj_gt):
    trj_gt01, trj_gt02 = split_trajectory(trj_gt)
    trj_gt01_sync, trj_est_sync = sync.associate_trajectories(trj_gt01, trj_est)
    trj_copy = copy.deepcopy(
        trj_est_sync
    )  # otherwise np arrays will be references and mess up stuff
    r_a, t_a, s = geometry.umeyama_alignment(
        trj_copy.positions_xyz.T, trj_gt01_sync.positions_xyz.T, False 
    )
    res = (lie.se3(r_a, t_a), trj_gt01, trj_gt02)
    return res, s


def split_trajectory(trj_gt:PoseTrajectory3D):
    traj_N = len(trj_gt.positions_xyz)
    trj_gt_01 = PoseTrajectory3D(
        trj_gt.positions_xyz[0:traj_N // 3 , :],
        trj_gt.orientations_quat_wxyz[0:traj_N // 3, :],
        trj_gt.timestamps[0:traj_N // 3],
    )
    trj_gt_02 = PoseTrajectory3D(
        trj_gt.positions_xyz[traj_N // 3:, :],
        trj_gt.orientations_quat_wxyz[traj_N // 3:, :],
        trj_gt.timestamps[traj_N // 3:],
    )
    return trj_gt_01, trj_gt_02


def evaluate_results(structvio_res_path, gt_path):
    ape_res = []
    trj_est = read_tum_trajectory_file(structvio_res_path)

    print("using truth:")
    print(gt_path)
    print("using vio truth:")
    print(structvio_res_path)
    # read the ground truth
    trj_gt = read_tum_trajectory_file(gt_path)
    res, s = align_trajectory(trj_est, trj_gt)
    trj_est.scale(s)
    trj_est.transform(res[0])

    trj1, trj2 = sync.associate_trajectories(trj_gt, trj_est)
    ape_res = ape(trj1, trj2, PoseRelation.translation_part)
    rpe_res = rpe(trj1,trj2, PoseRelation.translation_part, 1.0, Unit.frames)
    return ape_res, rpe_res, trj1, trj2


def plotResult(
    result: Result,
    traj_ref: PosePath3D,
    traj_est: PosePath3D,
    res_path: str,
    traj_ref_full: typing.Optional[PosePath3D] = None,
    plot_disable = True
) -> None:
    
    from evo.tools.settings import SETTINGS

    import matplotlib.pyplot as plt
    import numpy as np

    logger.debug(SEP)
    logger.debug("Plotting results... ")
    plot_mode = plot.PlotMode("xyz")

    # Plot the raw metric values.
    fig1 = plt.figure(figsize=SETTINGS.plot_figsize)

    x_array = result.np_arrays["seconds_from_start"]
    x_label = "$t$ (s)"

    plot.error_array(
        fig1.gca(),
        result.np_arrays["error_array"],
        x_array=x_array,
        statistics={
            s: result.stats[s]
            for s in SETTINGS.plot_statistics
            if s not in ("min", "max")
        },
        name=result.info["label"],
        title=result.info["title"],
        xlabel=x_label,
    )

    # Plot the values color-mapped onto the trajectory.
    fig2 = plt.figure(figsize=SETTINGS.plot_figsize)
    ax = plot.prepare_axis(fig2, plot_mode)

    plot.traj(
        ax,
        plot_mode,
        traj_ref_full if traj_ref_full else traj_ref,
        style=SETTINGS.plot_reference_linestyle,
        color=SETTINGS.plot_reference_color,
        label="reference",
        alpha=SETTINGS.plot_reference_alpha,
    )
    plot.draw_coordinate_axes(
        ax, traj_ref, plot_mode, SETTINGS.plot_reference_axis_marker_scale
    )

    plot_colormap_min = result.stats["min"]
    plot_colormap_max = result.stats["max"]

    plot.traj_colormap(
        ax,
        traj_est,
        result.np_arrays["error_array"],
        plot_mode,
        min_map=plot_colormap_min,
        max_map=plot_colormap_max,
        title=result.info["title"],
    )
    plot.draw_coordinate_axes(ax, traj_est, plot_mode, SETTINGS.plot_axis_marker_scale)
    if SETTINGS.plot_pose_correspondences:
        plot.draw_correspondence_edges(
            ax,
            traj_est,
            traj_ref,
            plot_mode,
            style=SETTINGS.plot_pose_correspondences_linestyle,
            color=SETTINGS.plot_reference_color,
            alpha=SETTINGS.plot_reference_alpha,
        )
    fig2.axes.append(ax)

    plot_collection = plot.PlotCollection(result.info["title"])
    plot_collection.add_figure("raw", fig1)
    plot_collection.add_figure("map", fig2)
    fig1.savefig(res_path + '/' + result.info["title"] + "_raw.png")
    fig2.savefig(res_path + '/' + result.info["title"] + "_map.png")
    if not plot_disable:
        plot_collection.show()
        plot_collection.close()
    
def run(reference:str, out_path:str):
    est_file = ""
    res_file = ""
    for root, dirs, files in os.walk(out_path):
        for file in files:
            if file.endswith(".tum"):
                est_file = os.path.join(root, file)

            if file.endswith(".json"):
                res_file = os.path.join(root, file)

    ape, rpe, trj1, trj2 = evaluate_results(est_file, reference)
    print("The loop closing errors are listed as \nAPE:")
    for key, val in ape.stats.items():
        print("\t" + key + "\t" + str(val))    
    print("RPE:")
    for key, val in rpe.stats.items():
        print("\t" + key + "\t" + str(val))    
    ape.info['title'] = "ape"
    plotResult(ape, trj1, trj2, out_path)
    rpe.info['title'] = "rpe"
    plotResult(rpe, trj1, trj2, out_path)
    update_res(ape, rpe, res_file)


if __name__ == "__main__":
    example_cmd = """
    %(prog)s -t gt.tum -o res 
    """

    parser = argparse.ArgumentParser(
        description="Evaluate the error of the slam/vio output.",
        usage=example_cmd,
    )

    parser.add_argument(
        "-r", "--reference", action="store", required=True, help="Path of the 'gt.tum'"
    )
    parser.add_argument(
        "-o", "--output", action="store", required=True, help="Path of the output folder"
    )
    parser.add_argument(
        "-p", "--plot_disable", default=False, action="store_true", required=False, help="Disable Plot"
    )
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(2)

    args = parser.parse_args()
    out_path = args.output
    est_file = ""
    res_file = ""
    for root, dirs, files in os.walk(out_path):
        for file in files:
            if file.endswith(".tum"):
                est_file = os.path.join(root, file)

            if file.endswith(".json"):
                res_file = os.path.join(root, file)

    ape, rpe, trj1, trj2 = evaluate_results(est_file, args.reference)
    print("The loop closing errors are listed as \nAPE:")
    for key, val in ape.stats.items():
        print("\t" + key + "\t" + str(val))    
    print("RPE:")
    for key, val in rpe.stats.items():
        print("\t" + key + "\t" + str(val))    
    ape.info['title'] = "ape"
    plotResult(ape, trj1, trj2, out_path, plot_disable=args.plot_disable)
    rpe.info['title'] = "rpe"
    plotResult(rpe, trj1, trj2, out_path, plot_disable=args.plot_disable)
    update_res(ape, rpe, res_file)
