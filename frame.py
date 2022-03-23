import numpy as np
import cv2
from math import sqrt
import utils
import circle
from circle import Circle
from redball import RedBall
from point import Point
from evaluation import Evaluation
import color_thresholding as ct


class Frame:
    image: np.array
    markers_pred: list
    markers_true: list
    MIN_BALL_SIZE: int

    def __init__(self, image):
        self.image = image
        self.markers_pred = []
        self.markers_true = []
        self.MIN_BALL_SIZE = 25

    def get_np_shape(self):
        return np.shape(self.image)

    def get_xy_shape(self):
        return np.shape(self.image)[1], np.shape(self.image)[0]

    def get_height(self):
        return np.shape(self.image)[0]

    def has_red_ball(self):
        return True if len(self.markers_pred) > 0 else False

    def has_red_ball_with_id(self, ball_id):
        for red_ball in self.markers_pred:
            if red_ball.ball_id == ball_id:
                return True
        return False

    def isABallInBottomHalf(self):
        return True if self.markers_pred[0].elevation < 0 else False

    def convert_to_contour(self, points):
        length = len(points)
        cnt = np.zeros([length, 1, 2])
        for i in range(length):
            cnt[i, 0, :] = points[i]
        return np.round(cnt).astype('int')

    def extract_red_ball_from_meta(self, meta):
        # json contours
        if "shapes" in meta.keys():
            for shape in meta["shapes"]:
                if shape["label"] == "redball":
                    cnt = self.convert_to_contour(shape["points"])
                    ball = circle.min_enclosing_circle(cnt)
                    if ball.radius >= self.MIN_BALL_SIZE:
                        self.markers_true.append(RedBall(ball, self.get_height()))
                        self.markers_true.sort()
        # txt yolo bbox
        if "BBox" in meta.keys():
            width, height = self.get_xy_shape()
            for bbox in meta["BBox"]:
                x = round(bbox[0]*width, 0)
                y = round(bbox[1]*height, 0)
                center = Point(x, y)
                radius = round(bbox[2]*width/2.0, 0)
                ball = Circle(center, radius)
                self.markers_pred.append(RedBall(ball, self.get_height()))
                self.markers_pred.sort()


    def extract_meta(self, meta):
        self.extract_red_ball_from_meta(meta)

    def calculate_post_top_centers(self, posts):
        post_top_centers = []
        for post in posts:
            post_top_centers.append(Point((post[0, 0] + post[1, 0]) / 2.0, (post[0, 1] + post[1, 1]) / 2.0))
        return post_top_centers

    def connectBallAndPost(self, posts, balls):
        post_top_centers = self.calculate_post_top_centers(posts)
        matched_balls = []
        for ball in balls:
            for post in post_top_centers:
                match = utils.check_close_and_below(ball.center, 3*ball.radius, post)
                if match:
                    matched_balls.append(ball)
        return matched_balls

    def find_white_posts_in_image(self):
        whiteMask = ct.HLS_white_mask(self.image)
        # dilation and erosion
        kernel = np.ones((11, 11), np.uint8)
        whiteMask = cv2.dilate(whiteMask, kernel, iterations=1)

        ret, thresh = cv2.threshold(whiteMask, 0, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        box_data = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                box = box[np.argsort(box[:, 1])]  # Sort by minimum y value
                box_data.append(box)
        return box_data

    def remove_duplication_predictions(self, post_pred):
        cleaned_post_pred = []
        for i in range(len(post_pred)):
            duplicate = False
            for j in range(i+1,len(post_pred)):
                duplicate = circle.approx_same(post_pred[i],post_pred[j])
                if duplicate:
                    break

            if not duplicate:
                cleaned_post_pred.append(RedBall(post_pred[i], self.get_height()))
        return cleaned_post_pred


    def find_red_balls_in_image(self):
        red_mask = ct.two_method_mask(self.image, ct.red_hue_mask, ct.excess_red_mask)
        contours, hierarchy = cv2.findContours(red_mask*255, cv2.RETR_EXTERNAL, 2)
        ball_hypotheses = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            ball = circle.min_enclosing_circle(cnt)
            if (ball.radius >= self.MIN_BALL_SIZE) and (area > 1200):
                ball_hypotheses.append(ball)

        post_hypotheses = self.find_white_posts_in_image()
        post_pred = self.connectBallAndPost(post_hypotheses, ball_hypotheses)
        self.markers_pred = self.remove_duplication_predictions(post_pred)
        self.markers_pred.sort()

    # Assume that there is no case of two trues fighting for one pred
    def evaluate_red_ball_predictions(self, saveImages=False):
        matches_t = np.zeros([len(self.markers_true)])
        matches_p = np.zeros([len(self.markers_pred)])
        for t in range(len(self.markers_true)):
            for p in range(len(self.markers_pred)):
                if circle.are_two_circles_overlapped(self.markers_true[t].ball, self.markers_pred[p].ball, iou_cutoff=0.5):
                    matches_t[t] += 1
                    matches_p[p] += 1
                    if saveImages:
                        cv2.circle(self.image, self.markers_pred[p].ball.get_center(), self.markers_pred[p].ball.radius, [0,255,0], 5)

        if saveImages:
            for p in range(len(self.markers_pred)):
                if matches_p[p] == 1:
                    cv2.circle(self.image, self.markers_pred[p].ball.get_center(), self.markers_pred[p].ball.radius, [0, 255, 0],
                               5)
                else:
                    cv2.circle(self.image, self.markers_pred[p].ball.get_center(), self.markers_pred[p].ball.radius, [0, 0, 255],
                               5)

        TP = np.sum(matches_t >= 1)
        FP = np.sum(matches_p == 0)
        FN = np.sum(matches_t == 0)
        print("TP: "+str(TP)+", FP: " + str(FP)+", FN: " + str(FN))
        return Evaluation(TP, FP, FN)

    def get_redball_elevation(self, ball_id: int):
        red_ball_match = None
        for red_ball in self.markers_pred:
            if red_ball.ball_id == ball_id:
                red_ball_match = red_ball.elevation
        return red_ball_match


