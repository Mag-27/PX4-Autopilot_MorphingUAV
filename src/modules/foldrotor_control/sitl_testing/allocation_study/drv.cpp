// Driver around the REAL FoldrotorAllocation. stdin: "Fx Fy Fz Mx My Mz" (body FLU) per line.
// stdout per line: F1 F2 a1 a2 b1 b2 sat. Arg "matrix" dumps M0 and Minv.
#include "/home/magesvarlinux/PX4-Autopilot/src/modules/foldrotor_control/FoldrotorAllocation.hpp"
#include <cstdio>
#include <cstring>
int main(int argc, char **argv)
{
	foldrotor::FoldrotorAllocation a;

	if (argc > 1 && !strcmp(argv[1], "matrix")) {
		for (int r = 0; r < 6; r++) { for (int c = 0; c < 6; c++) printf("%.9g ", (double)a.getM0()(r, c)); printf("\n"); }

		for (int r = 0; r < 6; r++) { for (int c = 0; c < 6; c++) printf("%.9g ", (double)a.getMinv()(r, c)); printf("\n"); }

		return 0;
	}

	float w[6];

	while (scanf("%f %f %f %f %f %f", &w[0], &w[1], &w[2], &w[3], &w[4], &w[5]) == 6) {
		auto o = a.allocate(matrix::Vector3f(w[0], w[1], w[2]), matrix::Vector3f(w[3], w[4], w[5]));
		printf("%.9g %.9g %.9g %.9g %.9g %.9g %d\n", (double)o.F1, (double)o.F2, (double)o.alpha1, (double)o.alpha2,
		       (double)o.beta1, (double)o.beta2, (int)o.saturated);
	}
}
