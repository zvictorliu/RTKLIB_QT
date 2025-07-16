/*------------------------------------------------------------------------------
 * rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
 *
 *          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
 *
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
 * history : 2007/01/16  1.0 new
 *           2007/03/15  1.1 add library mode
 *           2007/05/08  1.2 separate from postpos.c
 *           2009/01/20  1.3 support rtklib 2.2.0 api
 *           2009/12/12  1.4 support glonass
 *                           add option -h, -a, -l, -x
 *           2010/01/28  1.5 add option -k
 *           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
 *           2014/01/27  1.7 fix bug on default output time format
 *           2015/05/15  1.8 -r or -l options for fixed or ppp-fixed mode
 *           2015/06/12  1.9 output patch level in header
 *           2016/09/07  1.10 add option -sys
 *-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "rtklib.h"

#define PROGNAME "rnx2rtkp" /* program name */
#define MAXFILE 16          /* max number of input files */

/* help text -----------------------------------------------------------------*/
static const char *help[] = {
    "",
    " usage: rnx2rtkp [option]... file file [...]",
    "",
    " Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and ccompute ",
    " receiver (rover) positions and output position solutions.",
    " The first RINEX OBS file shall contain receiver (rover) observations. For the",
    " relative mode, the second RINEX OBS file shall contain reference",
    " (base station) receiver observations. At least one RINEX NAV/GNAV/HNAV",
    " file shall be included in input files. To use SP3 precise ephemeris, specify",
    " the path in the files. The extension of the SP3 file shall be .sp3 or .eph.",
    " All of the input file paths can include wild-cards (*). To avoid command",
    " line deployment of wild-cards, use \"...\" for paths with wild-cards.",
    " Command line options are as follows ([]:default). With -k option, the",
    " processing options are input from the configuration file. In this case,",
    " command line options precede options in the configuration file.",
    "",
    " -?        print help",
    " -k file   input options from configuration file [off]",
    " -o file   set output file [stdout]",
    " -ts ds ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
    " -te de te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
    " -ti tint  time interval (sec) [all]",
    " -p mode   mode (0:single,1:dgps,2:kinematic,3:static,4:moving-base,",
    "                 5:fixed,6:ppp-kinematic,7:ppp-static) [2]",
    " -m mask   elevation mask angle (deg) [15]",
    " -sys s[,s...] nav system(s) (s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) [G|R]",
    " -f freq   number of frequencies for relative mode (1:L1,2:L1+L2,3:L1+L2+L5) [2]",
    " -v thres  validation threshold for integer ambiguity (0.0:no AR) [3.0]",
    " -b        backward solutions [off]",
    " -c        forward/backward combined solutions [off]",
    " -i        instantaneous integer ambiguity resolution [off]",
    " -h        fix and hold for integer ambiguity resolution [off]",
    " -e        output x/y/z-ecef position [latitude/longitude/height]",
    " -a        output e/n/u-baseline [latitude/longitude/height]",
    " -n        output NMEA-0183 GGA sentence [off]",
    " -g        output latitude/longitude in the form of ddd mm ss.ss' [ddd.ddd]",
    " -t        output time in the form of yyyy/mm/dd hh:mm:ss.ss [sssss.ss]",
    " -u        output time in utc [gpst]",
    " -d col    number of decimals in time [3]",
    " -s sep    field separator [' ']",
    " -r x y z  reference (base) receiver ecef pos (m) [average of single pos]",
    "           rover receiver ecef pos (m) for fixed or ppp-fixed mode",
    " -l lat lon hgt reference (base) receiver latitude/longitude/height (deg/m)",
    "           rover latitude/longitude/height for fixed or ppp-fixed mode",
    " -y level  output soltion status (0:off,1:states,2:residuals) [0]",
    " -x level  debug trace level (0:off) [0]"};
/* show message --------------------------------------------------------------*/
extern int showmsg(char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    vfprintf(stderr, format, arg);
    va_end(arg);
    fprintf(stderr, "\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}

/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
    int i;
    for (i = 0; i < (int)(sizeof(help) / sizeof(*help)); i++)
        fprintf(stderr, "%s\n", help[i]);
    exit(0);
}

/* lzw adds*/
PyObject *pFunc_checksat = NULL, *pFunc_storeinfo= NULL, *pFunc_save= NULL, *pFunc_getval= NULL, *pFunc_checkvs= NULL;
PyObject *pModule = NULL;
PyObject *pName = NULL;
struct LZW_CONFIG lzw_config = {
    NULL, NULL, NULL, NULL, NULL,
    0, 0, 0, 0,
    1.0};

extern double get_coff(char *id, int week, int gpst)
{
    // 调用Python函数
    if (pFunc_getval && PyCallable_Check(pFunc_getval))
    {
        // 创建参数列表并传递给Python函数
        PyObject *pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(id)); // 字符参数
        PyTuple_SetItem(pArgs, 1, PyLong_FromLong(week));    // 整数参数1
        PyTuple_SetItem(pArgs, 2, PyLong_FromLong(gpst));    // 整数参数2

        // 调用Python函数
        PyObject *pValue = PyObject_CallObject(pFunc_getval, pArgs);
        Py_DECREF(pArgs);

        if (pValue != NULL)
        {
            double result = PyFloat_AsDouble(pValue);
            if (result > 0)
            {
                Py_DECREF(pValue);
                if (result != 1)
                {
                    trace(4, "[lzw] get_coff result=%f NLOS\n", result);
                }
                else
                {
                    trace(4, "[lzw] get_coff result=%f LOS\n", result);
                }
                return result;
            }
        }
        else
        {
            trace(2, "[lzw] get_coff pValue failed\n");
        }
    }
    else
    {
        trace(2, "[lzw] pFunc_getval failed\n");
    }

    return 1;
}

extern int is_NLOS(char *id, int week, int gpst)
{
    // 调用Python函数
    if (pFunc_checksat && PyCallable_Check(pFunc_checksat))
    {
        // 创建参数列表并传递给Python函数
        PyObject *pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(id)); // 字符参数
        PyTuple_SetItem(pArgs, 1, PyLong_FromLong(week));    // 整数参数1
        PyTuple_SetItem(pArgs, 2, PyLong_FromLong(gpst));    // 整数参数2

        // 调用Python函数
        PyObject *pValue = PyObject_CallObject(pFunc_checksat, pArgs);
        Py_DECREF(pArgs);

        if (pValue != NULL)
        {
            int result = PyLong_AsLong(pValue);
            if (result > 0)
            {
                Py_DECREF(pValue);
                trace(4, "[lzw] is_NLOS result=%d\n", result);
                return result;
            }
        }
        else
        {
            trace(2, "[lzw] is_NLOS pValue failed\n");
        }
    }
    else
    {
        trace(2, "[lzw] pFunc_checksat failed\n");
    }
    return 0;
}

extern int is_VS(char *id, int week, int gpst){

    trace(5, "is_VS(%s, %d, %d)\n", id, week, gpst);
    // 调用Python函数
    if (pFunc_checkvs && PyCallable_Check(pFunc_checkvs))
    {
        // 创建参数列表并传递给Python函数
        PyObject *pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(id)); // 字符参数
        PyTuple_SetItem(pArgs, 1, PyLong_FromLong(week));    // 整数参数1
        PyTuple_SetItem(pArgs, 2, PyLong_FromLong(gpst));    // 整数参数2

        // 调用Python函数
        PyObject *pValue = PyObject_CallObject(pFunc_checkvs, pArgs);
        Py_DECREF(pArgs);

        if (pValue != NULL)
        {
            int result = PyLong_AsLong(pValue);
            Py_DECREF(pValue);
            trace(5, "[lzw] is_vs result=%d\n", result);
            if (result > 0)
                return 1;
        }
        else
        {
            trace(2, "[lzw] is_VS pValue failed\n");
        }
    }
    else
    {
        trace(2, "[lzw] pFunc_checkvs failed\n");
    }
    return 0;
}

extern int store_info(gtime_t gtime, char *key, char *val)
{
    if (pFunc_storeinfo && PyCallable_Check(pFunc_storeinfo))
    {
        // 创建参数列表并传递给Python函数
        PyObject *pArgs = PyTuple_New(4);
        int week;
        double tow = time2gpst(gtime, &week);
        PyTuple_SetItem(pArgs, 0, PyLong_FromLong(week)); // 整数参数
        PyTuple_SetItem(pArgs, 1, PyFloat_FromDouble(tow)); // 整数参数
        PyTuple_SetItem(pArgs, 2, PyUnicode_FromString(key)); // 字符参数
        PyTuple_SetItem(pArgs, 3, PyUnicode_FromString(val)); // 字符参数

        // 调用Python函数
        PyObject *pValue = PyObject_CallObject(pFunc_storeinfo, pArgs);
        Py_DECREF(pArgs);

        if (pValue != NULL)
        {
            int result = PyLong_AsLong(pValue);
            if (result > 0)
            {
                Py_DECREF(pValue);
                trace(5, "[lzw] store_info result=%d\n", result);
                return result;
            }
        }
        else
        {
            trace(2, "[lzw] store_info pValue failed\n");
        }
    }
    else
    {
        trace(2, "[lzw] pFunc_storeinfo failed\n");
    }

    return -1;
}


/* rnx2rtkp main -------------------------------------------------------------*/
int main(int argc, char **argv)
{
    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = {""};
    gtime_t ts = {0}, te = {0};
    double tint = 0.0, es[] = {2000, 1, 1, 0, 0, 0}, ee[] = {2000, 12, 31, 23, 59, 59}, pos[3];
    int i, j, n, ret;
    char *infile[MAXFILE], *outfile = "", *p;

    prcopt.mode = PMODE_KINEMA;
    prcopt.navsys = 0;
    prcopt.refpos = 1;
    prcopt.glomodear = 1;
    solopt.timef = 0;
    sprintf(solopt.prog, "%s ver.%s %s", PROGNAME, VER_RTKLIB, PATCH_LEVEL);
    sprintf(filopt.trace, "%s.trace", PROGNAME);

    /* load options from configuration file */
    for (i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-k") && i + 1 < argc)
        {
            resetsysopts();
            if (!loadopts(argv[++i], sysopts))
                return -1;
            getsysopts(&prcopt, &solopt, &filopt);
        }
    }
    for (i = 1, n = 0; i < argc; i++)
    {
        if (!strcmp(argv[i], "-o") && i + 1 < argc)
            outfile = argv[++i];
        else if (!strcmp(argv[i], "-ts") && i + 2 < argc)
        {
            sscanf(argv[++i], "%lf/%lf/%lf", es, es + 1, es + 2);
            sscanf(argv[++i], "%lf:%lf:%lf", es + 3, es + 4, es + 5);
            ts = epoch2time(es);
        }
        else if (!strcmp(argv[i], "-te") && i + 2 < argc)
        {
            sscanf(argv[++i], "%lf/%lf/%lf", ee, ee + 1, ee + 2);
            sscanf(argv[++i], "%lf:%lf:%lf", ee + 3, ee + 4, ee + 5);
            te = epoch2time(ee);
        }
        else if (!strcmp(argv[i], "-ti") && i + 1 < argc)
            tint = atof(argv[++i]);
        else if (!strcmp(argv[i], "-k") && i + 1 < argc)
        {
            ++i;
            continue;
        }
        else if (!strcmp(argv[i], "-p") && i + 1 < argc)
            prcopt.mode = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-f") && i + 1 < argc)
            prcopt.nf = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-sys") && i + 1 < argc)
        {
            for (p = argv[++i]; *p; p++)
            {
                switch (*p)
                {
                case 'G':
                    prcopt.navsys |= SYS_GPS;
                case 'R':
                    prcopt.navsys |= SYS_GLO;
                case 'E':
                    prcopt.navsys |= SYS_GAL;
                case 'J':
                    prcopt.navsys |= SYS_QZS;
                case 'C':
                    prcopt.navsys |= SYS_BDS;
                case 'I':
                    prcopt.navsys |= SYS_IRN;
                }
                if (!(p = strchr(p, ',')))
                    break;
            }
        }
        else if (!strcmp(argv[i], "-m") && i + 1 < argc)
            prcopt.elmin = atof(argv[++i]) * D2R;
        else if (!strcmp(argv[i], "-v") && i + 1 < argc)
            prcopt.thresar[0] = atof(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc)
            strcpy(solopt.sep, argv[++i]);
        else if (!strcmp(argv[i], "-d") && i + 1 < argc)
            solopt.timeu = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-b"))
            prcopt.soltype = 1;
        else if (!strcmp(argv[i], "-c"))
            prcopt.soltype = 2;
        else if (!strcmp(argv[i], "-i"))
            prcopt.modear = 2;
        else if (!strcmp(argv[i], "-h"))
            prcopt.modear = 3;
        else if (!strcmp(argv[i], "-t"))
            solopt.timef = 1;
        else if (!strcmp(argv[i], "-u"))
            solopt.times = TIMES_UTC;
        else if (!strcmp(argv[i], "-e"))
            solopt.posf = SOLF_XYZ;
        else if (!strcmp(argv[i], "-a"))
            solopt.posf = SOLF_ENU;
        else if (!strcmp(argv[i], "-n"))
            solopt.posf = SOLF_NMEA;
        else if (!strcmp(argv[i], "-g"))
            solopt.degf = 1;
        else if (!strcmp(argv[i], "-r") && i + 3 < argc)
        {
            prcopt.refpos = prcopt.rovpos = 0;
            for (j = 0; j < 3; j++)
                prcopt.rb[j] = atof(argv[++i]);
            matcpy(prcopt.ru, prcopt.rb, 3, 1);
        }
        else if (!strcmp(argv[i], "-l") && i + 3 < argc)
        {
            prcopt.refpos = prcopt.rovpos = 0;
            for (j = 0; j < 3; j++)
                pos[j] = atof(argv[++i]);
            for (j = 0; j < 2; j++)
                pos[j] *= D2R;
            pos2ecef(pos, prcopt.rb);
            matcpy(prcopt.ru, prcopt.rb, 3, 1);
        }
        else if (!strcmp(argv[i], "-y") && i + 1 < argc)
            solopt.sstat = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-x") && i + 1 < argc)
            solopt.trace = atoi(argv[++i]);
        else if (*argv[i] == '-')
            printhelp();
        else if (n < MAXFILE)
            infile[n++] = argv[i];
    }
    if (!prcopt.navsys)
    {
        prcopt.navsys = SYS_GPS | SYS_GLO;
    }
    if (n <= 0)
    {
        showmsg("error : no input file");
        return -2;
    }

    /* lzw adds */

    // 初始化Python解释器
    Py_Initialize();
    // 导入当前路径
    PyRun_SimpleString("import  sys");
    PyRun_SimpleString("import  os");
    PyRun_SimpleString("sys.path.append(os.getcwd())");
    // 模块路径
    PyRun_SimpleString("sys.path.append('/home/zongwei/ws/projects/gnss_analyze/apps/positioning')");
    PyRun_SimpleString("sys.path.append('/home/zongwei/ws/projects/gnss_analyze')");
    
    pName = PyUnicode_DecodeFSDefault("asset");
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    if (pModule != NULL)
    {
        pFunc_checksat = PyObject_GetAttrString(pModule, "check_sat");
        pFunc_checkvs = PyObject_GetAttrString(pModule, "check_vs");
        pFunc_storeinfo = PyObject_GetAttrString(pModule, "store_info");
        pFunc_save = PyObject_GetAttrString(pModule, "save_info");
        pFunc_getval = PyObject_GetAttrString(pModule, "get_val");
        Py_DECREF(pModule);
        trace(1, "pModule succeed\n");

        if (pFunc_checksat && PyCallable_Check(pFunc_checksat))
        {
            trace(1, "pFunc_checksat succeed\n");
        }
        else
        {
            trace(1, "pFunc_checksat failed\n");
            PyErr_Print();
        }

        if (pFunc_checkvs && PyCallable_Check(pFunc_checkvs))
        {
            trace(1, "pFunc_checkvs succeed\n");
        }
        else
        {
            trace(1, "pFunc_checkvs failed\n");
            PyErr_Print();
        }

        if (pFunc_storeinfo && PyCallable_Check(pFunc_storeinfo))
        {
            trace(1, "pFunc_storeinfo succeed\n");
        }
        else
        {
            trace(1, "pFunc_storeinfo failed\n");
            PyErr_Print();
        }

        if (pFunc_save && PyCallable_Check(pFunc_save))
        {
            trace(1, "pFunc_save succeed\n");
        }
        else
        {
            trace(1, "pFunc_save failed\n");
            PyErr_Print();
        }

        if (pFunc_getval && PyCallable_Check(pFunc_getval))
        {
            trace(1, "pFunc_getval succeed\n");
        }
        else
        {
            trace(1, "pFunc_getval failed\n");
            PyErr_Print();
        }
    }
    else
    {
        trace(1, "pModule failed\n");
        PyErr_Print();
    }
    // 读取环境变量
    lzw_config.nlos_env = getenv("NLOS_ENV");
    lzw_config.varr_env = getenv("VARR_ENV");
    lzw_config.vs_env = getenv("VS");
    lzw_config.k_env = getenv("K_COFF");
    lzw_config.ar_env = getenv("AR_MODES");

    if (lzw_config.nlos_env != NULL)
    {
        lzw_config.nlos_val = atoi(lzw_config.nlos_env);
    }
    if (lzw_config.varr_env != NULL)
    {
        lzw_config.varr_val = atoi(lzw_config.varr_env);
    }
    if (lzw_config.vs_env != NULL)
    {
        lzw_config.vs_val = atoi(lzw_config.vs_env);
    }
    if (lzw_config.k_env != NULL)
    {
        lzw_config.k_val = atof(lzw_config.k_env);
    }
    if (lzw_config.ar_env != NULL)
    {
        lzw_config.ar_val = atoi(lzw_config.ar_env);
    }
    // coff_val = 1;
    if (lzw_config.nlos_val == 1)
    {
        trace(1, "[nlos_val]: %d NLOS sats excluded\n", lzw_config.nlos_val);
    }
    else{
        trace(1, "[nlos_val]: %d NLOS sats included\n", lzw_config.nlos_val);
    }
    switch (lzw_config.varr_val)
    {
    case 0:
        trace(1, "traditional varr\n");
        break;
    default:
        trace(1, "[varr_val]: %d \n", lzw_config.varr_val);
        break;
    }
    trace(1, "[k_val]: %.1f\n", lzw_config.k_val);
    trace(1, "[vs_val]: %d\n", lzw_config.vs_val);
    if (lzw_config.ar_val == 0)
    {
        trace(1, "AR including NLOS\n");
    }
    else if (lzw_config.ar_val == 1)
    {
        trace(1, "AR excluding NLOS\n");
    }

    
    ret = postpos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infile, n, outfile, "", "");

    // 保存信息
    if (pFunc_save && PyCallable_Check(pFunc_save)){
        // 调用Python函数
        PyObject_CallObject(pFunc_save, NULL);
    }

    // 释放Python解释器
    Py_DECREF(pFunc_checksat);
    Py_DECREF(pFunc_storeinfo);
    Py_DECREF(pFunc_save);
    Py_DECREF(pFunc_getval);
    Py_Finalize();

    if (!ret)
        fprintf(stderr, "%40s\r", "");
    return ret;
}
