import sys
import pptk
import logging
import argparse
import laspy as lp
import numpy as np
from pathlib import Path
from typing import List
from typing import Optional

from coloredlogs import ColoredFormatter


def recursive_split(x_min, y_min, x_max, y_max, max_x_size, max_y_size):
    """
    :param x_min:
    :param y_min:
    :param x_max:
    :param y_max:
    :param max_x_size:
    :param max_y_size:
    :return:
    """
    x_size = x_max - x_min
    y_size = y_max - y_min

    if x_size > max_x_size:
        left = recursive_split(x_min, y_min, x_min + (x_size // 2), y_max, max_x_size, max_y_size)
        right = recursive_split(x_min + (x_size // 2), y_min, x_max, y_max, max_x_size, max_y_size)
        return left + right
    elif y_size > max_y_size:
        up = recursive_split(x_min, y_min, x_max, y_min + (y_size // 2), max_x_size, max_y_size)
        down = recursive_split(x_min, y_min + (y_size // 2), x_max, y_max, max_x_size, max_y_size)
        return up + down
    else:
        return [(x_min, y_min, x_max, y_max)]


def optimized_reading():
    """
    Source:
        - https://laspy.readthedocs.io/en/latest/examples.html

    :return:
    """
    with lp.open(sys.argv[1]) as file:
        sub_bounds = recursive_split(
            file.header.x_min,
            file.header.y_min,
            file.header.x_max,
            file.header.y_max,
            args.size[0],
            args.size[1]
        )

        writers: List[Optional[lp.LasWriter]] = [None] * len(sub_bounds)
        try:
            count = 0
            for points in file.chunk_iterator(args.points_per_iter):
                logging.info(f"{count / file.header.point_count * 100}%")

                x, y = points.x.copy(), points.y.copy()

                point_piped = 0

                for i, (x_min, y_min, x_max, y_max) in enumerate(sub_bounds):
                    mask = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)

                    if np.any(mask):
                        if writers[i] is None:
                            output_path = Path(sys.argv[2]) / f"output_{i}.laz"
                            writers[i] = lp.open(output_path, mode='w', header=file.header)
                        sub_points = points[mask]
                        writers[i].write_points(sub_points)

                    point_piped += np.sum(mask)
                    if point_piped == len(points):
                        break
                count += len(points)
            logging.info(f"{count / file.header.point_count * 100}%")
        finally:
            for writer in writers:
                if writer is not None:
                    writer.close()


def main(arguments):
    """
    Sources:
        - https://pypi.org/project/laspy/
        - https://towardsdatascience.com/guide-to-real-time-visualisation-of-massive-3d-point-clouds-in-python-ea6f00241ee0
        - https://towardsdatascience.com/how-to-automate-lidar-point-cloud-processing-with-python-a027454a536c
        - https://github.com/heremaps/pptk
        - https://laspy.readthedocs.io/en/latest/examples.html

    :param arguments:
    :return:
    """
    lidar_filepath = arguments.lidar_filepath

    # with lp.open(lidar_filepath) as f:
    #     print(f"Point format:       {f.header.point_format}")
    #     print(f"Number of points:   {f.header.point_count}")
    #     print(f"Number of vlrs:     {len(f.header.vlrs)}")

    # logging.info(">> Reading {} lidar file...".format(lidar_filepath))
    # with lp.open(lidar_filepath) as fh:
    #     logging.info('Points from Header:', fh.header.point_count)
    #     las = fh.read()
    #     logging.info(las)
    #     logging.info('Points from data:', len(las.points))
    #     ground_pts = las.classification == 2
    #     bins, counts = np.unique(las.return_number[ground_pts], return_counts=True)
    #     logging.info('Ground Point Return Number distribution:')
    #     for r, c in zip(bins, counts):
    #         logging.info('    {}:{}'.format(r, c))

    point_cloud = lp.read(lidar_filepath)

    logging.info(">>>> LAS file info:")
    logging.info(">>>>>> Header: {}".format(point_cloud.header))
    logging.info(">>>>>> Point format: {}".format(point_cloud.header.point_format))
    logging.info(">>>>>> Number of points: {}".format(point_cloud.header.point_count))
    logging.info(">>>>>> Classes: {}".format(set(list(point_cloud.classification))))

    logging.info("Sampling 1st return...")
    first = lp.create(point_format=point_cloud.header.point_format, file_version=point_cloud.header.version)
    first.points = point_cloud.points[point_cloud.classification == 2]
    first.write('first.las')

    # logging.info(">>>> Transposing x,y,z points in numpy...")
    # points = np.stack([point_cloud.X, point_cloud.Y, point_cloud.Z], axis=0).transpose((1, 0))
    # points_intensity = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()

    # factor = 10
    # decimated_points_random = points[::factor]

    # show point cloud
    # v = pptk.viewer()

    # with intensities
    # v.attributes(points_intensity / 65535)

    # managing the point size, putting the background black and not displaying the grid and axis information
    # v.color_map('cool')
    # v.set(point_size=0.001, bg_color=[0, 0, 0, 0], show_axis=0, show_grid=0)

    # spliting classes
    # buildings = laspy.create(point_format=las.header.point_format, file_version=las.header.version)
    # buildings.points = las.points[las.classification == 6]


if __name__ == '__main__':
    """    
        DESCRIPTION

        Optional arguments:
          -h, --help             Show this help message and exit
          -f, -lidar_filepath    DESCRIPTION                
          -v, -verbose           Boolean to logging.info output logging or not

         Usage: 
            > python main.py [-h] [-f LIDAR_FILEPATH] [-v VERBOSE] 

         Example:
            > python main.py -f /media/rodolfo/data/lidar/NP_T-0358_FWF.las -v True
        """
    parser = argparse.ArgumentParser(description='DESCRIPTION')
    parser.add_argument('-f', '-lidar_filepath', action="store", dest='lidar_filepath', help='DESCRIPTION')
    parser.add_argument('-v', '-verbose', action="store", dest='verbose', help='logging.info log of processing')
    args = parser.parse_args()

    if eval(args.verbose):
        log = logging.getLogger('')
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(logging.INFO)
        cf = ColoredFormatter("[%(asctime)s] {%(filename)-15s:%(lineno)-4s} %(levelname)-5s: %(message)s ")
        ch.setFormatter(cf)
        log.addHandler(ch)

        fh = logging.FileHandler('logging.log')
        fh.setLevel(logging.INFO)
        ff = logging.Formatter("[%(asctime)s] {%(filename)-15s:%(lineno)-4s} %(levelname)-5s: %(message)s ",
                               datefmt='%Y.%m.%d %H:%M:%S')
        fh.setFormatter(ff)
        log.addHandler(fh)

        log.setLevel(logging.DEBUG)
    else:
        logging.basicConfig(format="%(levelname)s: %(message)s")

    main(args)


