#include "ImageSourceDir.h"

ImageSourceDir::ImageSourceDir(const char* directory, bool _keepFrames)
{
	curImage = NULL;

	m_keepFrames = _keepFrames;

	strncpy (m_DirSpec, directory, strlen(directory)+1);

	countFiles();
	setCurrentImage(0);
}

ImageSourceDir::~ImageSourceDir()
{	
	if(!m_keepFrames && curImage)
		cvReleaseImage (&curImage);
		
	// release all pictures in map
	for(map<string,IplImage*>::iterator it2 = m_loadedImagesMap.begin(); it2 != m_loadedImagesMap.end(); it2++)
		cvReleaseImage (&(it2->second));			

	m_loadedImagesMap.clear();
	
}

// get IPL Image
void ImageSourceDir::getIplImage()
{

	// check current File Counter
	if ( m_curFile >= m_numFiles || m_curFile == -1)
	{
		curImage = NULL;
        m_curFile = -1;
		return;
	}

	// assemble image filename
	char tmpStr[MAX_PATH+1];
	sprintf (tmpStr, "%s/%s", m_DirSpec, m_fileNames[m_curFile].c_str());

	// check if image was already loaded, then return it, 
	// else load it and store it into a vector of images
	string fn(tmpStr);
	if(m_keepFrames)
	{
		if(!m_loadedImagesMap[fn])
		{
			m_loadedImagesMap[fn] = cvLoadImage(tmpStr, -1);
		}

		// assign
		curImage = m_loadedImagesMap[fn];
	}
	else
	{
		if(curImage)
			cvReleaseImage (&curImage);
		curImage = cvLoadImage(tmpStr, -1);
	}
	// increase current File Counter
	m_curFile++;

}

// get IPL Image depending on IDx
void ImageSourceDir::getIplImage(int idx)
{
	//printf("GetIPL!\n");
	// Index check
	if (idx >= m_numFiles || idx < 0)
	{
		curImage = NULL;
		return;
	}

	setCurrentImage(idx);

	// assemble image filename
	char tmpStr[MAX_PATH+1];
	sprintf (tmpStr, "%s/%s", m_DirSpec, m_fileNames[m_curFile].c_str());

	// check if image was already loaded, then return it, 
	// else load it and store it into a vector of images
	string fn(tmpStr);
	if(m_keepFrames)
	{
		if(m_loadedImagesMap[fn])
		{
			cvReleaseImage(&(m_loadedImagesMap[fn]));
		}

		// reload
		m_loadedImagesMap[fn] = cvLoadImage(tmpStr, -1);

		// assign
		curImage = m_loadedImagesMap[fn];
	}
	else
	{
		if(curImage)
			cvReleaseImage (&curImage);
		curImage = cvLoadImage(tmpStr, -1);
	}

	// increase current File Counter
	m_curFile++;
}


void ImageSourceDir::getIplImage(const std::string& fileName)
{

	// assemble image filename
	char tmpStr[MAX_PATH+1];
	sprintf (tmpStr, "%s/%s", m_DirSpec, fileName.c_str());

	// find corresponding idx
	int idx = -1;
	for (int i = 0; i < m_numFiles; i++) {
		if( fileName.compare(m_fileNames[i]) == 0 ) {
			idx = i;
			break;
		}
	}
	if (idx >= m_numFiles || idx < 0)
	{
		curImage = NULL;
		return;
	}
	// we set IDX to current Image Position
	setCurrentImage(idx);

	// check if image was already loaded, then return it, 
	// else load it and store it into a vector of images
	string fn(tmpStr);
	if(m_keepFrames)
	{
		if(m_loadedImagesMap[fn])
		{
			cvReleaseImage(&(m_loadedImagesMap[fn]));
		}

		// reload
		m_loadedImagesMap[fn] = cvLoadImage(tmpStr, -1);

		// assign
		curImage = m_loadedImagesMap[fn];
	}
	else
	{
		if(curImage)
			cvReleaseImage (&curImage);
		curImage = cvLoadImage(tmpStr, -1);
	}

	// increase current File Counter
	m_curFile++;
}

void ImageSourceDir::reloadIplImage()
{    	
	// assemble image filename
	char tmpStr[MAX_PATH+1];
	sprintf (tmpStr, "%s/%s", m_DirSpec, m_fileNames[m_curFile-1].c_str());

	// check if image was already loaded, then return it, 
	// else load it and store it into a vector of images
	string fn(tmpStr);
	if(m_keepFrames)
	{
		if(m_loadedImagesMap[fn])
			cvReleaseImage(&(m_loadedImagesMap[fn]));
		// reload
		m_loadedImagesMap[fn] = cvLoadImage(tmpStr, -1);
		
		// assign
		curImage = m_loadedImagesMap[fn];
	}
	else
	{
		if(curImage)
			cvReleaseImage (&curImage);
		curImage = cvLoadImage(tmpStr, -1);
	}
}


bool ImageSourceDir::setCurrentImage(int idx)
{
	if (idx >= m_numFiles || idx < 0) 
		return false; 

	m_curFile = idx;
	return true;
}


int ImageSourceDir::countFiles()
{
	m_numFiles=0;
	m_fileNames.clear();


	char tmpDirSpec[MAX_PATH+1];
	sprintf (tmpDirSpec, "%s/*", m_DirSpec);
#if OS_type==2
	WIN32_FIND_DATA f;
	HANDLE h = FindFirstFile(tmpDirSpec , &f);
	if(h != INVALID_HANDLE_VALUE)
	{
		FindNextFile(h, &f);	//read ..
		FindNextFile(h, &f);	//read .
		do
		{
			m_fileNames.push_back(f.cFileName);
		} while(FindNextFile(h, &f));
		m_numFiles = m_fileNames.size();
	}
	FindClose(h);
#endif

#if OS_type==1

    //directory
    QDir dir(m_DirSpec);

    //filters for names to open (imahe types)
    QStringList name_filters;
    name_filters << "*.jpeg" << "*.jpg" << "*.png" << "*.bmp";    

    //extract filenames
    QStringList filenames = dir.entryList(name_filters, QDir::Files, QDir::Name);

    m_numFiles = filenames.size();

    for (int i = 0 ; i < filenames.size(); ++i)
    {
        m_fileNames[i] = QString(filenames[i]).toStdString();

    }



#endif
	return m_numFiles;

}

const char* ImageSourceDir::getFilename(int idx)
{
	if (idx >= m_numFiles) return NULL; 
	if (idx < 0) idx = m_curFile;

	return m_fileNames[idx].c_str();
}


int ImageSourceDir::getNumImages()
{
    return m_numFiles;
}

void ImageSourceDir::setRandomImage()
{
	setCurrentImage( (int)(m_numFiles * rand()/( RAND_MAX+1.0f )) );
}

// returns the index of the current image
int ImageSourceDir::getCurrentImageIndex()
{
	// indizes start from zero 
	// if there is no image
	if (curImage == NULL)
		return m_curFile;
	else
		// else we have to decrease
		return m_curFile-1; 
}
