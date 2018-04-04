conf = {
    ecm = {
        mask = { size = 5, nb_mask = 180 },
        range = { init = 8, tracking = 10 },
        contrast = { edge_threshold = 5000, mu1 = 0.5, mu2 = 0.5 }
        },
    sample = {
        step = 2,
        nb_sample = 500
        },
    camera = {
        u0 = 640, v0 = 360,
        px = 2000, py = 2000
        },
    rendering = {
        edgeRend_threshold = 20,
        clipDist = 1.1,
        sampleRend = 4,
        useNPoints = 0,
        nPoints = 700,
        scaleModel = 1,
        xDir = 1,
        yDir = 1,
        zDir = 1
        },
    trackingtype = 4,
      --weightme = 0.0,
    weightme = 0.5,
    --weightccd = 0.0000015,
    --weightklt = 10,
    weightccd = 0.5,
     --weightccd = 0.0,
     weightklt = 0.0,
     ccd = {
        gamma_1 = 0.5,
        gamma_2 = 5,
        gamma_3 = 7,
        gamma_4 = 5,
        alpha = 1.2,
        beta = 0.06,
        kappa = 0.5,
        c = 0.25,
        h = 15,
        delta_h = 1,
        resolution = 300,
        degree = 4,
        phi_dim = 6
        },
     klt = {
        blocksize = 12,
        npoints = 500,
        history = 25,
        windowsize = 18,
        quality = 0.000001,
        mindist = 8,
        harrisfree = 0.01,
        useharris = 1,
        pyramidlevels = 1
     },
    computecovariance = 1,
    usekalman = 1,
    kalman = {
    --sigmaqt = 0.04,
    --sigmaqr = 0.001,
    sigmaqt = 0.0015,
    sigmaqr = 0.0004,
    sigmapt = 0.002,
    sigmapr = 0.0001},
    detection = {
        similarity = 0,
        nbimax = 100,
        startinglevel = 1,
        nscales = 10,
        sample = 5,
                pfilter = {
                nbparticles = 100,
                lambda = 200
                },
                canny = {
                cannyTh1 = 150,
                cannyTh2 = 210
                },
                shapecontext = {
                nradius = 20,
                ntheta = 40,
                samplex = 20,
                sampley = 20,
                sampletheta = 1,
                overlap = 1
                },
                orientedchamfer = {
                mud = 300,
                lambdao = 0.7
                },
                bayesian = {
                sigmaf = 0.2,
                }
        },
        segmentation = {
        klt = {
        blocksize = 12,
        npoints = 800,
        history = 25,
        windowsize = 10,
        quality = 0.0001,
        mindist = 10,
        harrisfree = 0.1,
        useharris = 1,
        pyramidlevels = 1,
        nbfeaturesmax = 400,
        motionfilter = 20,
        gridbg = 20,
        gridfg = 40
        },
        energy = {
        alpha0 = 0.6,
        beta0 = 0.3,
        gamma0 = 0.7,
        alpha = 0.3,
        beta = 0.5,
        gamma = 0.9
        },
        kernel = {
        bwidthcol0 = 255,
        bwidthcol1 = 4000,
        bwidthspat = 550
        },
        ransac = {
        pointstobuildp = 7,
        backgroundthresh = 0.8,
        nmaxiterations = 1000,
        consensus = 0.5,
        minimalsizemodel = 200
        },
        backgroundtype = 0,
        startframe = 15,
        deltahomography = 5,
        ngaussians = 5,
        nbins = 10
        },
      learn = {
        dist = 50,
        similarityL = 0,
        srho = 1,
        stheta = 45,
        sphi = 45,
        srtheta = 8,
        srphi = 8,
        overlap = 1
        }
    }
