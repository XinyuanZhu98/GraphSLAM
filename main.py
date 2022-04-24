
from slam_classes import *
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--meas", default="data_preprocessed.dat",
        help=".dat file containing odometric and laser measuements")
    parser.add_argument(
        "--init", default="SLAM_graph_nodes_odom.dat",
        help="initial guess")
    parser.add_argument(
        "--max_iter", default=100,
        help="maximum number of iterations allowed")
    parser.add_argument(
        "--tol", default=0.1,
        help="tolerance for convergence check")
    parser.add_argument(
        "--output_txt", default=r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/result_odom/",
        help="directory to place the output .txt file recording the node info")
    parser.add_argument(
        "--output_plot", default=r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/figure_odom/",
        help="directory to place the output plots of the trajectories")

    args = parser.parse_args()

    # Create an object of SLAMGraph
    SLAM_graph = SLAMGraph()

    # Read data files
    SLAM_graph.read_data(meas_file=args.meas, node_file=args.init)

    # Graph optimization
    norm_dz, norm_err = SLAM_graph.optimize(args.output_txt, args.output_plot,
                                            max_iterations=int(args.max_iter),
                                            tolerance=float(args.tol))

    # Plot graph convergence
    plt.figure(1)
    plt.plot(norm_dz, color='darkcyan')
    plt.title('SLAM Graph Updates per Iteration')
    plt.ylabel('|dz|')
    plt.xlabel('iteration')
    plt.grid()
    plt.savefig('dz.png')
    plt.clf()

    # Plot error evolution
    plt.figure(2)
    plt.plot(norm_err, color='darkcyan')
    plt.title('SLAM Graph Error per Iteration')
    plt.ylabel('|error|')
    plt.xlabel('iteration')
    plt.grid()
    plt.savefig('error.png')
    plt.clf()

    # Plot the optimized graph
    plt.figure(3)
    SLAM_graph.output_res(title='Optimized Graph',
                          path_txt=args.output_txt,
                          path_fig=args.output_plot,)
    plt.clf()




