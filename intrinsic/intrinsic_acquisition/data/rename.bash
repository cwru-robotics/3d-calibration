x_min=1.85;
y_min=-0.22;
z_min=1.0;
dx=0.05;
dy=0.11;
dz=0.1;

for f in image_*.png; do

	x=${f:6:1};
	y=${f:8:1};
	z=${f:10:1};
	
	x_out=$(calc -p -- $x_min+$x*$dx);
	y_out=$(calc -p -- $y_min+$y*$dy);
	z_out=$(calc -p -- $z_min+$z*$dz);
	
	
	mv -v "$f" "img_${x_out}_${y_out}_${z_out}.png";
done
for f in img_*.png; do
	mv -v "$f" "${f/./p}";
done
for f in img_*.png; do
	mv -v "$f" "${f/./p}";
done
for f in img_*.png; do
	mv -v "$f" "${f/./p}";
done
for f in img_*ppng; do
	mv -v "$f" "${f/ppng/.png}"
done
