
rand_num() {
	min="$1"
	max="$2"
	MAX_RAND="100000"
	rn=$(shuf -i 1-${MAX_RAND} -n 1)
	echo "(${rn}/${MAX_RAND}) * (${max} - ${min}) + ${min}" | bc -l

}

curr_dir=$(pwd)/
dp_dir=$(pwd)/..
py_dir="${dp_dir}/examples/ML4KP"
cpp_dir="${dp_dir}/src/cpp/bin/ML4KP/"

total=100
pi="3.1415926536"
pi2="6.2831853072"
for i in $(seq 0 ${total});
do 
	th0=$(rand_num 0.0 ${pi2})
	th1=$(rand_num 0.0 ${pi2})
	dth0=$(rand_num -15.0 15.0)
	dth1=$(rand_num -15.0 15.0)

	x0="${th0} ${th1} ${dth0} ${dth1}"
	x0c="${th0},${th1},${dth0},${dth1}"
	# echo "x0: ${x0}"
	python3.11 ${py_dir}/trajectory.py --plant acrobot --time 1 --video True --dir "${curr_dir}/out" --x0 "${x0}" 
	${cpp_dir}/trajectory --time=1 --filename="${curr_dir}/out/cpp_traj.txt" --plant=acrobot --start="[${x0c}]"

	python3.11 check_traj_files.py --file1 "${curr_dir}/out/py_traj.txt" --file2 "${curr_dir}/out/cpp_traj.txt"
done;