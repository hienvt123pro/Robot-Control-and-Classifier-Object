""" create Excel file"""

# from openpyxl import Workbook
# from find_feature import find_features
# from preprocessing import preprocessing_img
# import cv2
#
#
# workbook = Workbook()
# sheet1 = workbook.active
# sheet1.title = "size"
# sheet1['A1'] = 'd1'
# sheet1['B1'] = 'd2'
# sheet1['C1'] = 'd3'
# sheet1['D1'] = 'y'
#
#
# def create_workbook(path, index, d_1, d_2, d_3, y):
#     col1name = 'A'
#     col2name = 'B'
#     col3name = 'C'
#     col4name = 'D'
#     col1name = col1name + str(index)
#     col2name = col2name + str(index)
#     col3name = col3name + str(index)
#     col4name = col4name + str(index)
#
#     sheet1[col1name] = d_1
#     sheet1[col2name] = d_2
#     sheet1[col3name] = d_3
#     sheet1[col4name] = y
#     workbook.save(path)
#
#
# # size 16
# txt16 = 'dataset/trainning/size16/dep16 ({0}).jpg'
# for i in range(26):
#     img = cv2.imread(txt16.format(i))
#     img_org, img_contour = preprocessing_img(img)
#     image, center, d1, d2, d3 = find_features(img_org, img_contour)
#     create_workbook("size.xlsx", i + 2, d1, d2, d3, 0)
#
# # size 18
# txt18 = 'dataset/trainning/size18/dep18 ({0}).jpg'
# for i in range(26):
#     img = cv2.imread(txt18.format(i))
#     img_org, img_contour = preprocessing_img(img)
#     image, center, d1, d2, d3 = find_features(img_org, img_contour)
#     create_workbook("size.xlsx", i + 28, d1, d2, d3, 1)
#
# # size 20
# txt20 = 'dataset/trainning/size20/dep20 ({0}).jpg'
# for i in range(26):
#     img = cv2.imread(txt20.format(i))
#     img_org, img_contour = preprocessing_img(img)
#     image, center, d1, d2, d3 = find_features(img_org, img_contour)
#     create_workbook("size.xlsx", i + 54, d1, d2, d3, 2)
#
# # not size
# txt_error = 'dataset/trainning/error/error ({0}).jpg'
# for i in range(26):
#     img = cv2.imread(txt_error.format(i))
#     img_org, img_contour = preprocessing_img(img)
#     image, center, d1, d2, d3 = find_features(img_org, img_contour)
#     create_workbook("size.xlsx", i + 80, d1, d2, d3, 3)
