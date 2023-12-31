#include <iostream>
#include <vector>
#include "cstatebased.hpp"
#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define M2NM 0.0005399568034557236
#define NM2M 1852.0
#define KTS2MS 0.514444
#define FT2M 0.3048
#define FPM2MS 0.00508

static PyObject* cstatebased_detect(PyObject* self, PyObject* args)
{
    PyObject *ownship = NULL,
             *intruder = NULL,
             *pRPZ = NULL,
             *pHPZ = NULL,
             *ptlookahead = NULL;

    if (!PyArg_ParseTuple(args, "OOOOO", &ownship, &intruder, &pRPZ, &pHPZ, &ptlookahead))
        return NULL;

    PyDoubleArrayAttr lat1(ownship, "lat"),  lon1(ownship, "lon"),  trk1(ownship, "trk"),
                      gs1 (ownship, "gs"),   alt1(ownship, "alt"),  vs1 (ownship, "vs"),
                      lat2(intruder, "lat"), lon2(intruder, "lon"), trk2(intruder, "trk"),
                      gs2 (intruder, "gs"),  alt2(intruder, "alt"), vs2 (intruder, "vs");

    PyListAttr  acid(ownship, "id");

    PyDoubleArrayAttr rpz(pRPZ), hpz(pHPZ), tlook(ptlookahead);
    // Only continue if all arrays exist
    if (lat1 && lon1 && trk1 && gs1  && alt1 && vs1  && lat2 && lon2 && trk2 && gs2  && alt2 && vs2 && rpz && hpz && tlook)
    {
        // Assume all arrays are the same size; only get the size of lat1
        npy_intp  size  = lat1.size();

        // Loop over all combinations of aircraft to detect conflicts
        conflict confhor, confver;
        double tin, tout;
        double dalt, dvs;

        npy_bool acinconf = NPY_FALSE;
        double tcpamax_ac = 0.0;

        // rpz, hpz, and tlook pointers for both sides
        double *rpz1 = rpz.ptr_start;
        double *rpz2 = rpz.ptr_start;
        double *hpz1 = hpz.ptr_start;
        double *hpz2 = hpz.ptr_start;
        double *tlook1 = tlook.ptr_start;
        double *tlook2 = tlook.ptr_start;
        double tlook_cur;
        // Return values
        PyDoubleArrayAttr tcpamax(size);
        PyBoolArrayAttr inconf(size);
        PyListAttr confpairs, lospairs, qdr, dist, dcpa, tcpa, tinconf;

        for (unsigned int i = 0; i < size; ++i) {
            acinconf = NPY_FALSE;
            for (unsigned int j = 0; j < size; ++j) {
                if (i != j) {
                    // Vectical detection first
                    dalt = *alt1.ptr - *alt2.ptr;
                    dvs  = *vs1.ptr  - *vs2.ptr;
                    tlook_cur = std::max(*tlook1, *tlook2);
                    if (detect_ver(confver, std::max(*hpz1, *hpz2), tlook_cur, dalt, dvs)) {
                        // Horizontal detection
                        if (detect_hor(confhor, std::max(*rpz1, *rpz2), tlook_cur,
                                       *lat1.ptr * DEG2RAD, *lon1.ptr * DEG2RAD, *gs1.ptr, *trk1.ptr * DEG2RAD,
                                       *lat2.ptr * DEG2RAD, *lon2.ptr * DEG2RAD, *gs2.ptr, *trk2.ptr * DEG2RAD))
                        {
                            tin  = std::max(confhor.tin, confver.tin);
                            tout = std::min(confhor.tout, confver.tout);
                            // Combined conflict?
                            if (tin <= tlook_cur && tin < tout && tout > 0.0) {
                                // Add AC id to conflict list
                                PyObject* pair = PyTuple_Pack(2, acid[i], acid[j]);
                                confpairs.append(pair);
                                tcpamax_ac = std::max(confhor.tcpa, tcpamax_ac);
                                acinconf = NPY_TRUE; // This aircraft is in conflict
                                if (confver.LOS && confhor.LOS) {
                                    // Add to lospairs if this is also a LoS
                                    lospairs.append(pair);
                                }
                                Py_DECREF(pair);
                                qdr.append(confhor.q * RAD2DEG);
                                dist.append(confhor.d);
                                dcpa.append(confhor.dcpa);
                                tcpa.append(confhor.tcpa);
                                tinconf.append(tin);
                            }
                        }
                    }
                }
                rpz2++; hpz2++; tlook2++;
                lat2.ptr++; lon2.ptr++; trk2.ptr++; gs2.ptr++; alt2.ptr++; vs2.ptr++;
            }
            *inconf.ptr = acinconf;
            *tcpamax.ptr = tcpamax_ac;
            inconf.ptr++;
            tcpamax.ptr++;
            acinconf = NPY_FALSE;
            tcpamax_ac = 0.0;
            lat2.ptr = lat2.ptr_start; lon2.ptr = lon2.ptr_start;
            trk2.ptr = trk2.ptr_start; gs2.ptr  = gs2.ptr_start;
            alt2.ptr = alt2.ptr_start; vs2.ptr  = vs2.ptr_start;
            rpz2 = rpz.ptr_start;
            hpz2 = hpz.ptr_start;
            tlook2 = tlook.ptr_start;
            rpz1++; hpz1++; tlook1++;
            lat1.ptr++; lon1.ptr++; trk1.ptr++; gs1.ptr++; alt1.ptr++; vs1.ptr++;
        }

        return PyTuple_Pack(9, confpairs.attr, lospairs.attr, inconf.arr, tcpamax.arr, qdr.attr, dist.attr, dcpa.attr, tcpa.attr, tinconf.attr);
    }

    Py_RETURN_NONE;
};

static PyMethodDef methods[] = {
    {"detect", cstatebased_detect, METH_VARARGS, "Detect conflicts for traffic"},
    {NULL}  /* Sentinel */
};

#ifndef PyMODINIT_FUNC  /* declarations for DLL import/export */
#define PyMODINIT_FUNC void
#endif
#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef cstatebased_def =
{
    PyModuleDef_HEAD_INIT,
    "cstatebased",     /* name of module */
    "",          /* module documentation, may be NULL */
    -1,          /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    methods
};

PyMODINIT_FUNC PyInit_cstatebased(void)
{
    import_array();
    return PyModule_Create(&cstatebased_def);
};
#else
PyMODINIT_FUNC initcstatebased(void)
{
    Py_InitModule("cstatebased", methods);
    import_array();
};
#endif
