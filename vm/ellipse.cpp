#include "ellipse.h"

ellipse::ellipse(int instance, int target) {
	vm_state *vm= bin_factory(instance);
    vm->step();

    Complex speed_N_2, speed_N, speed;
    Complex pos_N, pos;
    apexs = apexf = Complex();
    while (apexf == Complex() || apexs == Complex()) {
        vm->step();
        pos_N = pos;
        speed_N_2 = speed_N;
        speed_N = speed;
        pos = vm->get_target_absolute_position(target);
        speed = vm->get_targets_speeds()[target];
        if (speed_N_2 != Complex()) {
            if (abs(speed_N_2) < abs(speed_N) && abs(speed_N) > abs(speed))
                apexf = pos_N;
            if (abs(speed_N_2) > abs(speed_N) && abs(speed_N) < abs(speed))
                apexs =pos_N;
        }
    }
    center = (apexf + apexs) / Complex(2.,0);
    semi_major_axis = apexf - center;
    excentricity = abs(center) / abs(semi_major_axis);
    period = 2. * M_PI * sqrt(abs(semi_major_axis)*abs(semi_major_axis)*abs(semi_major_axis) / MU);
    double periastre, apoastre;
    periastre = abs(apexf - apexs) / 2.;
    apoastre = sqrt((1. - (excentricity * excentricity)) * abs(semi_major_axis) * abs(semi_major_axis));
    radii = Complex(periastre, apoastre);

	delete vm;
}

/*Complex ellipse::get_speed(Complex oriented_pos) {
    Complex pos = oriented_pos - center;
    double dM = sqrt(MU / real(radii)*real(radii)*real(radii));
    double vx = -((real(radii) / abs(pos)) / sqrt(1 - excentricity*excentricity) * imag(pos) * dM);
    double vy = real(radii) / abs(pos) * sqrt(1 - excentricity * excentricity) * (real(pos) + real(radii) * excentricity) * dM;
    Complex speed(vx, vy);

    return speed + center;
}*/

double ellipse::distance(Complex oriented_pos) {
    Complex pos = oriented_pos - center;
    double factor = (real(pos) * real(pos)) / (real(radii) * real(radii)) +
                    (imag(pos) * imag(pos)) / (imag(radii) * imag(radii));

    return factor;
}
