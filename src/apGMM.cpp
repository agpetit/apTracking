/*
 * GrabCut implementation source code 
 * by Justin Talbot, jtalbot@stanford.edu
 * Placed in the Public Domain, 2010
 * 
 */

#include "apGMM.h"

apGMM::apGMM(unsigned int K) : m_K(K)
{

	m_gaussians = new Gaussian[m_K];

}

apGMM::~apGMM()
{
	if (m_gaussians)
		delete [] m_gaussians;
}

double apGMM::p(vpRGBa c)
{
	double result = 0;

	if (m_gaussians)
	{
		for (unsigned int i=0; i < m_K; i++)
		{
			result += m_gaussians[i].pi * p(i, c);
		}
	}

	return result;
}

double apGMM::p(unsigned int i, vpRGBa c)
{
	double result = 0;

	if( m_gaussians[i].pi > 0 )
	{
		if (m_gaussians[i].determinant > 0)
		{
			double r = c.R - m_gaussians[i].mu.R;
			double g = c.G - m_gaussians[i].mu.G;
			double b = c.B - m_gaussians[i].mu.B;
			
			double d = r * (r*m_gaussians[i].inverse[0][0] + g*m_gaussians[i].inverse[1][0] + b*m_gaussians[i].inverse[2][0]) +
					g * (r*m_gaussians[i].inverse[0][1] + g*m_gaussians[i].inverse[1][1] + b*m_gaussians[i].inverse[2][1]) +
					b * (r*m_gaussians[i].inverse[0][2] + g*m_gaussians[i].inverse[1][2] + b*m_gaussians[i].inverse[2][2]);

			result = (double)(1.0/(sqrt(m_gaussians[i].determinant)) * exp(-0.5*d));
		}
	}

	return result;
}

void apGMM::buildGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, std::vector<vpColVector> &colBgd, std::vector<vpColVector> &colFgd, std::vector<int> &componentBack, std::vector<int> &componentFore)
{

	// Step 3: Build GMMs using Orchard-Bouman clustering algorithm

	// Set up Gaussian Fitters
	GaussianFitter* backFitters = new GaussianFitter[backgroundGMM.K()];
	GaussianFitter* foreFitters = new GaussianFitter[foregroundGMM.K()];

	unsigned int foreCount = 0, backCount = 0;

	// Initialize the first foreground and background clusters
	for (int i = 0; i<colBgd.size(); i++)
	{
	vpRGBa col;
	col.R = colBgd[i][0];
	col.G = colBgd[i][1];
	col.B = colBgd[i][2];
	backFitters[0].add(col);
	backCount++;
	componentBack[i] = 0;
	}

	for (int i = 0; i<colFgd.size(); i++)
	{
	vpRGBa col;
	col.R = colFgd[i][0];
	col.G = colFgd[i][1];
	col.B = colFgd[i][2];
	foreFitters[0].add(col);
	foreCount++;
	componentFore[i] = 0;
	}



	backFitters[0].finalize(backgroundGMM.m_gaussians[0], backCount, true);
	foreFitters[0].finalize(foregroundGMM.m_gaussians[0], foreCount, true);

	std::cout << " ok 3 " << (double)foregroundGMM.m_gaussians[0].eigenvectors[0][0] << " " << (double)foregroundGMM.m_gaussians[0].eigenvectors[0][1] << " " << (double)foregroundGMM.m_gaussians[0].eigenvectors[0][2] << std::endl;


	unsigned int nBack = 0, nFore = 0;		// Which cluster will be split
	unsigned int maxK = backgroundGMM.K() > foregroundGMM.K() ? backgroundGMM.K() : foregroundGMM.K();
	

	// Compute clusters
	for (unsigned int i = 1; i < maxK; i++)
	{
		// Reset the fitters for the splitting clusters
		backFitters[nBack] = GaussianFitter();
		foreFitters[nFore] = GaussianFitter();

		// For brevity, get references to the splitting Gaussians
		Gaussian& bg = backgroundGMM.m_gaussians[nBack];
		Gaussian& fg = foregroundGMM.m_gaussians[nFore];

		// Compute splitting points
		double splitBack = bg.eigenvectors[0][0] * bg.mu.R + bg.eigenvectors[1][0] * bg.mu.G + bg.eigenvectors[2][0] * bg.mu.B;
		double splitFore = fg.eigenvectors[0][0] * fg.mu.R + fg.eigenvectors[1][0] * fg.mu.G + fg.eigenvectors[2][0] * fg.mu.B;

		// Split clusters nBack and nFore, place split portion into cluster i
		for (int j = 0; j<colFgd.size(); j++)
		{
			vpRGBa c;
			c.R = colFgd[j][0];
			c.G = colFgd[j][1];
			c.B = colFgd[j][2];
			if (i < foregroundGMM.K() && componentFore[j] == nFore)
				if (fg.eigenvectors[0][0] * c.R + fg.eigenvectors[1][0] * c.G + fg.eigenvectors[2][0] * c.B > splitFore)
				{
					componentFore[j] = i;
					foreFitters[i].add(c);
				}
				else
				{
					foreFitters[nFore].add(c);
				}
		}

		for (int j = 0; j<colBgd.size(); j++)
		{
			vpRGBa c;
			c.R = colBgd[j][0];
			c.G = colBgd[j][1];
			c.B = colBgd[j][2];
			if (i < backgroundGMM.K() && componentBack[j] == nBack)
				if (bg.eigenvectors[0][0] * c.R + bg.eigenvectors[1][0] * c.G + bg.eigenvectors[2][0] * c.B > splitBack)
				{
					componentBack[j] = i;
					backFitters[i].add(c);
				}
				else
				{
					backFitters[nBack].add(c);
				}
		}

		
		// Compute new split Gaussians
		backFitters[nBack].finalize(backgroundGMM.m_gaussians[nBack], backCount, true);
		foreFitters[nFore].finalize(foregroundGMM.m_gaussians[nFore], foreCount, true);

		if (i < backgroundGMM.K())
			backFitters[i].finalize(backgroundGMM.m_gaussians[i], backCount, true);
		if (i < foregroundGMM.K())
			foreFitters[i].finalize(foregroundGMM.m_gaussians[i], foreCount, true);

		// Find clusters with highest eigenvalue
		nBack = 0;
		nFore = 0;

		for (unsigned int j = 0; j <= i; j++ )
		{
			if (j < backgroundGMM.K() && backgroundGMM.m_gaussians[j].eigenvalues[0] > backgroundGMM.m_gaussians[nBack].eigenvalues[0])
				nBack = j;

			if (j < foregroundGMM.K() && foregroundGMM.m_gaussians[j].eigenvalues[0] > foregroundGMM.m_gaussians[nFore].eigenvalues[0])
				nFore = j;
		}

		std::cout << " ok 3 " << (double)foregroundGMM.m_gaussians[i].eigenvectors[0][0] << " " << (double)foregroundGMM.m_gaussians[i].eigenvectors[0][1] << " " << (double)foregroundGMM.m_gaussians[i].eigenvectors[0][2] << std::endl;

	}

	delete [] backFitters;
	delete [] foreFitters;
}

void apGMM::learnGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, int *components, const vpImage<vpRGBa>& image, int *label)
{
	// Step 4: Assign each pixel to the component which maximizes its probability
	for (unsigned int y = 0; y < image.getHeight(); ++y)
	{
		for (unsigned int x = 0; x < image.getWidth(); ++x)
		{
			vpRGBa c = image[y][x];

			if (label[y + x*image.getHeight()] == 1)
			{
				int k = 0;
				double max = 0;

				for (unsigned int i = 0; i < foregroundGMM.K(); i++)
				{
					double p = foregroundGMM.p(i, c);
					if (p > max)
					{
						k = i;
						max = p;
					}
				}

				components[y + x*image.getHeight()] = k;
			}
			else 
			{
				int k = 0;
				double max = 0;

				for (unsigned int i = 0; i < backgroundGMM.K(); i++)
				{
					double p = backgroundGMM.p(i, c);
					if (p > max)
					{
						k = i;
						max = p;
					}
				}

				components[y + x*image.getHeight()] = k;
			}
		}
	}

	// Step 5: Relearn GMMs from new component assignments

	// Set up Gaussian Fitters
	GaussianFitter* backFitters = new GaussianFitter[backgroundGMM.K()];
	GaussianFitter* foreFitters = new GaussianFitter[foregroundGMM.K()];

	unsigned int foreCount = 0, backCount = 0;

	for (unsigned int y = 0; y < image.getHeight(); ++y)
	{
		for(unsigned int x = 0; x < image.getWidth(); ++x)
		{
			vpRGBa c = image[y][x];

			if(label[y + x*image.getHeight()] == 1)
			{
				foreFitters[components[y + x*image.getHeight()]].add(c);
				foreCount++;
			}
			else
			{
				backFitters[components[y + x*image.getHeight()]].add(c);
				backCount++;
			}
		}
	}

	for (unsigned int i = 0; i < backgroundGMM.K(); i++)
		backFitters[i].finalize(backgroundGMM.m_gaussians[i], backCount, false);

	for (unsigned int i = 0; i < foregroundGMM.K(); i++)
		foreFitters[i].finalize(foregroundGMM.m_gaussians[i], foreCount, false);

	delete [] backFitters;
	delete [] foreFitters;
}


// GaussianFitter functions
GaussianFitter::GaussianFitter()
{
	/*s.R = 0;
	s.G = 0;
	s.B = 0;*/

	s[0] = 0;
	s[1] = 0;
	s[2] = 0;

	p.resize(3,3);

	p[0][0] = 0; p[0][1] = 0; p[0][2] = 0;
	p[1][0] = 0; p[1][1] = 0; p[1][2] = 0;
	p[2][0] = 0; p[2][1] = 0; p[2][2] = 0;

	count = 0;
}

// Add a color sample
void GaussianFitter::add(vpRGBa c)
{
	//s.R += c.R; s.G += c.G; s.B += c.B;
	s[0] += c.R; s[1] += c.G; s[2] += c.B;

	//std::cout << (double)s[0] << " " << (double)s[1] << " " << (double)s[2] << std::endl;

	p[0][0] += (double)c.R*c.R; p[0][1] += (double)c.R*c.G; p[0][2] += (double)c.R*c.B;
	p[1][0] += (double)c.G*c.R; p[1][1] += (double)c.G*c.G; p[1][2] += (double)c.G*c.B;
	p[2][0] += (double)c.B*c.R; p[2][1] += (double)c.B*c.G; p[2][2] += (double)c.B*c.B;

	count++;
}

// Build the gaussian out of all the added colors
void GaussianFitter::finalize(Gaussian& g, unsigned int totalCount, bool computeEigens) const
{
	// Running into a singular covariance matrix is problematic. So we'll add a small epsilon
	// value to the diagonal elements to ensure a positive definite covariance matrix.

	/*g.covariance.resize(3,3);
	g.inverse.resize(3,3);
	g.eigenvalues.resize(3);
	g.eigenvectors.resize(3,3);*/

	const double Epsilon = 0.0001;

	if (count==0)
	{
		g.pi = 0;
	}
	else
	{
		// Compute mean of gaussian
		/*g.mu.R = (double)s.R/count;
		g.mu.G = (double)s.G/count;
		g.mu.B = (double)s.B/count;*/

		g.mu.R = s[0]/count;
		g.mu.G = s[1]/count;
		g.mu.B = s[2]/count;

		// Compute covariance matrix
		g.covariance[0][0] = p[0][0]/count-(double)(g.mu.R*g.mu.R) + Epsilon; g.covariance[0][1] = p[0][1]/count-(double)(g.mu.R*g.mu.G); g.covariance[0][2] = p[0][2]/count-(double)(g.mu.R*g.mu.B);
		g.covariance[1][0] = p[1][0]/count-(double)(g.mu.G*g.mu.R); g.covariance[1][1] = p[1][1]/count-(double)(g.mu.G*g.mu.G) + Epsilon; g.covariance[1][2] = p[1][2]/count-(double)(g.mu.G*g.mu.B);
		g.covariance[2][0] = p[2][0]/count-(double)(g.mu.B*g.mu.R); g.covariance[2][1] = p[2][1]/count-(double)(g.mu.B*g.mu.G); g.covariance[2][2] = p[2][2]/count-(double)(g.mu.B*g.mu.B) + Epsilon;

		// Compute determinant of covariance matrix
		g.determinant = g.covariance[0][0]*(g.covariance[1][1]*g.covariance[2][2]-g.covariance[1][2]*g.covariance[2][1]) 
						- g.covariance[0][1]*(g.covariance[1][0]*g.covariance[2][2]-g.covariance[1][2]*g.covariance[2][0]) 
						+ g.covariance[0][2]*(g.covariance[1][0]*g.covariance[2][1]-g.covariance[1][1]*g.covariance[2][0]);

		// Compute inverse (cofactor matrix divided by determinant)
		g.inverse[0][0] =  (g.covariance[1][1]*g.covariance[2][2] - g.covariance[1][2]*g.covariance[2][1]) / g.determinant;
		g.inverse[1][0] = -(g.covariance[1][0]*g.covariance[2][2] - g.covariance[1][2]*g.covariance[2][0]) / g.determinant;
		g.inverse[2][0] =  (g.covariance[1][0]*g.covariance[2][1] - g.covariance[1][1]*g.covariance[2][0]) / g.determinant;
		g.inverse[0][1] = -(g.covariance[0][1]*g.covariance[2][2] - g.covariance[0][2]*g.covariance[2][1]) / g.determinant;
		g.inverse[1][1] =  (g.covariance[0][0]*g.covariance[2][2] - g.covariance[0][2]*g.covariance[2][0]) / g.determinant;
		g.inverse[2][1] = -(g.covariance[0][0]*g.covariance[2][1] - g.covariance[0][1]*g.covariance[2][0]) / g.determinant;
		g.inverse[0][2] =  (g.covariance[0][1]*g.covariance[1][2] - g.covariance[0][2]*g.covariance[1][1]) / g.determinant;
		g.inverse[1][2] = -(g.covariance[0][0]*g.covariance[1][2] - g.covariance[0][2]*g.covariance[1][0]) / g.determinant;
		g.inverse[2][2] =  (g.covariance[0][0]*g.covariance[1][1] - g.covariance[0][1]*g.covariance[1][0]) / g.determinant;

		// The weight of the gaussian is the fraction of the number of pixels in this Gaussian to the number of 
		// pixels in all the gaussians of this GMM.
		g.pi = (double)count/totalCount;

		if (computeEigens)
		{

#ifdef USE_DOUBLE
	#define CV_TYPE CV_64FC1
#else
	#define CV_TYPE CV_32FC1
#endif
			// Build OpenCV wrappers around our data.
			CvMat mat = cvMat(3, 3, CV_TYPE, g.covariance);
			CvMat eval = cvMat(3, 1, CV_TYPE, g.eigenvalues);
			CvMat evec = cvMat(3, 3, CV_TYPE, g.eigenvectors);
			
			// Compute eigenvalues and vectors using SVD
			cvSVD( &mat, &eval, &evec );

			double* data;
			double* data1;
			int step,step1;
			std::cout << " ok " << std::endl;
			for(int k = 0; k < 3; k++)
			{
				data1 = eval.data.db;
				step1 = eval.step/sizeof(double);
				g.eigenvalues[k] = (data1)[k];
				std::cout << " eig0 " << g.eigenvalues[k] << std::endl;
				for(int l = 0; l < 3 ; l++  )
				{
					data = evec.data.db;
					step = evec.step/sizeof(double);
					g.eigenvectors[k][l] = (data + k*step)[l];
					std::cout << " eig " << g.eigenvectors[k][l] << std::endl;
				}
			}
		}
	}
}
