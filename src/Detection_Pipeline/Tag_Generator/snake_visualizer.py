import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.transforms import Affine2D
from pathlib import Path
import cv2

# from Tag_Generator import generate_apriltag
from april_tag_gen import generate_apriltag



class SnakeVisualizer:

    def __init__(self, args):

        self.num_tags = args.num_tags
        self.link_length = args.link_length
        self.tag_size = args.tag_size
        self.fps = args.fps


        #
        # Generate AprilTags
        #
        self.tag_images = []

        for tag_id in range(self.num_tags):

            generate_apriltag(
                tag_id=tag_id,
                size=200,
            )

            path = (
                Path.cwd()
                /
                "tags"
                /
                f"tag_{tag_id}_200.png"
            )

            img = cv2.imread(
                str(path),
                cv2.IMREAD_GRAYSCALE,
            )

            self.tag_images.append(
                img
            )


        #
        # Visualization
        #
        self.fig, self.ax = plt.subplots(
            figsize=(8,5)
        )

        self.ax.set_aspect("equal")

        # Calculate required visualization size
        snake_length = (self.num_tags - 1) * self.link_length

        margin = 0.5

        self.ax.set_xlim(
            -margin,
            snake_length + margin
        )

        self.ax.set_ylim(
            -(snake_length / 2) - margin,
            (snake_length / 2) + margin
        )

        self.ax.set_title(
            "AprilTag Snake Visualization"
        )


        self.links = []

        for _ in range(self.num_tags-1):

            line, = self.ax.plot(
                [],
                [],
                linewidth=3,
            )

            self.links.append(line)



        #
        # Actual AprilTag images
        #
        self.tags = []

        for i in range(self.num_tags):

            img = self.tag_images[i]

            tag = self.ax.imshow(
                img,
                cmap="gray",
                extent=[
                    -self.tag_size/2,
                    self.tag_size/2,
                    -self.tag_size/2,
                    self.tag_size/2,
                ],
                interpolation="nearest",
                zorder=10,
            )

            self.tags.append(tag)



    def forward_kinematics(self,t):

        positions = np.zeros(
            (self.num_tags,2)
        )

        angles = np.zeros(
            self.num_tags
        )


        for i in range(1,self.num_tags):

            angles[i] = (
                0.8 *
                np.sin(
                    t*2+i*0.5
                )
            )


            direction = np.array(
                [
                    np.cos(angles[i]),
                    np.sin(angles[i]),
                ]
            )


            positions[i] = (
                positions[i-1]
                +
                self.link_length
                *
                direction
            )


        return positions, angles



    def update(self,frame):

        t = frame/self.fps


        positions,angles = (
            self.forward_kinematics(t)
        )


        #
        # Update rigid links
        #
        for i,line in enumerate(self.links):

            p1=positions[i]
            p2=positions[i+1]

            line.set_data(
                [
                    p1[0],
                    p2[0]
                ],
                [
                    p1[1],
                    p2[1]
                ]
            )



        #
        # Move and rotate AprilTags
        #
        for i,tag in enumerate(self.tags):

            x,y = positions[i]

            angle = np.degrees(
                angles[i]
            )


            transform = (
                Affine2D()
                .rotate_deg(angle)
                .translate(x,y)
                +
                self.ax.transData
            )


            tag.set_transform(
                transform
            )


        return (
            self.links
            +
            self.tags
        )



    def run(self):

        animation = FuncAnimation(
            self.fig,
            self.update,
            interval=1000/self.fps,
            blit=False,
        )

        plt.show()



def main(args):

    visualizer = SnakeVisualizer(args)

    visualizer.run()



if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Visualize an AprilTag snake.")
    parser.add_argument("--num_tags", type=int, default=6, help="Number of AprilTags.")
    parser.add_argument( "--link_length", type=float, default=0.25, help="Distance between adjacent tags [m].")
    parser.add_argument( "--tag_size", type=float, default=0.10, help="Displayed tag size [m].")
    parser.add_argument("--fps", type=int, default=30, help="Animation frame rate.")

    main(parser.parse_args())