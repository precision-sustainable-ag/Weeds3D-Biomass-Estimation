



file_in = open('.\\data\\Aprilbundle.out', 'r')
file_out = open('.\\data\\Aprilbundle.ply', 'w')

Lines = file_in.readlines()
num_cameras = int(Lines[1].split()[0])
num_points = int(Lines[1].split()[1])
print(num_cameras)
print(num_points)

file_out.write("ply\n")
file_out.write("format ascii 1.0\n")
file_out.write("element vertex " + str(num_points) + "\n")
file_out.write("property float x\n")
file_out.write("property float y\n")
file_out.write("property float z\n")
file_out.write("property uchar red\n")
file_out.write("property uchar green\n")
file_out.write("property uchar blue\n")
file_out.write("end_header\n")

for i in range(num_cameras*5+2, num_cameras*5+2 + num_points*3, 3):
    linexyz = Lines[i].split()
    linergb = Lines[i+1].split()
    x = float(linexyz[0])
    y = float(linexyz[1])
    z = float(linexyz[2])
    r = int(linergb[0])
    g = int(linergb[1])
    b = int(linergb[2])
    outline = str(x) + " " + str(y) + " " + str(z) + " " + str(r) + " " + str(g) + " " + str(b) + "\n"
    file_out.write(outline)

file_in.close()
file_out.close()