function mat4 = mat4fromvec7(vec7)
%mat4fromvec7(vec7)
%Returns a 4x4 transformation matrix representing the pose in vec7
%where vec7 is [posx posy posz quatw quatx quaty quatz]
position = vec7(1:3);
quaternion = vec7(4:7);
mat4 = eye(4); mat4(1:3, 4) = position;
mat4 = mat4 * mat4fromquat(quaternion);

function mat4 = mat4fromquat(q)
w = q(1); x = q(2); y = q(3); z = q(4);
ww = w*w;
xx = x*x;
yy = y*y;
zz = z*z;
mat4 = [...
    ww + xx - yy - zz       2 * (x*y - w*z)         2 * (x*z + w*y)     0;
    2 * (x*y + w*z)         ww - xx + yy - zz       2 * (y*z - w*x)     0;
    2 * (x*z - w*y)         2 * (y*z + w*x)         ww - xx - yy + zz   0;
    0                       0                       0                   1];