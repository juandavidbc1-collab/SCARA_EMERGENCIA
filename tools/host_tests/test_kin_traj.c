/* Pruebas en PC de cinemática.  Compilar: ver tools/host_tests/run.sh */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "kinematics.h"

static int fallos = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FALLO: " __VA_ARGS__); printf("\n"); fallos++; } } while (0)
#define D2R(d) ((d) * (float)M_PI / 180.0f)

int main(void)
{
    kin_cfg_t c = { .l1 = 160.0f, .l2 = 160.0f, .sing_margin = D2R(5.0f) };
    float qmin[2] = { D2R(-120), D2R(-140) }, qmax[2] = { D2R(120), D2R(140) };

    /* 1) FK(IK(p)) = p en una malla dentro del espacio de trabajo */
    int n = 0, ok = 0, lim = 0, sing = 0, inal = 0;
    float emax = 0;
    for (float x = -340; x <= 340; x += 10)
        for (float y = -340; y <= 340; y += 10) {
            float q[2]; int cod;
            ik_status_t s = kin_ik(&c, x, y, +1, qmin, qmax, q, &cod);
            n++;
            if (s == IK_OK) {
                ok++;
                float xr, yr; kin_fk(&c, q[0], q[1], &xr, &yr);
                float e = hypotf(xr - x, yr - y);
                if (e > emax) emax = e;
                CHECK(e < 0.05f, "FK(IK) error %.4f mm en (%g,%g)", e, x, y);
                CHECK(q[0] >= qmin[0] && q[0] <= qmax[0] && q[1] >= qmin[1] && q[1] <= qmax[1], "limites");
            } else if (s == IK_LIMITES) lim++;
            else if (s == IK_SINGULAR) sing++;
            else inal++;
        }
    printf("IK malla: %d puntos -> ok %d, limites %d, singular %d, inalcanzable %d; error FK(IK) max = %.5f mm\n",
           n, ok, lim, sing, inal, emax);

    /* 2) casos puntuales */
    float q[2]; int cod;
    CHECK(kin_ik(&c, 400, 0, 1, qmin, qmax, q, &cod) == IK_INALCANZABLE, "400,0 debe ser inalcanzable");
    CHECK(kin_ik(&c, 319.9f, 0, 1, qmin, qmax, q, &cod) == IK_SINGULAR, "borde exterior debe ser singular");
    CHECK(kin_ik(&c, 0, 0, 1, qmin, qmax, q, &cod) == IK_SINGULAR, "origen (brazo plegado) singular");
    CHECK(kin_ik(&c, 200, 100, 1, qmin, qmax, q, &cod) == IK_OK && cod == 1 && q[1] > 0, "codo +");
    CHECK(kin_ik(&c, 200, 100, -1, qmin, qmax, q, &cod) == IK_OK && cod == -1 && q[1] < 0, "codo -");
    /* límites que solo permiten un codo: q2 en [0,140] -> pedir codo - debe caer al codo + */
    float qmin2[2] = { D2R(-120), D2R(0) };
    CHECK(kin_ik(&c, 200, 100, -1, qmin2, qmax, q, &cod) == IK_OK && cod == 1, "debe conmutar de codo");
    float qmin3[2] = { D2R(100), D2R(0) }, qmax3[2] = { D2R(120), D2R(10) };
    CHECK(kin_ik(&c, 200, 100, 1, qmin3, qmax3, q, &cod) == IK_LIMITES, "debe violar limites");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
