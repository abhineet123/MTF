#ifndef IMAGE_SOURCE_DIR_H
#define IMAGE_SOURCE_DIR_H

#include "ImageSource.h"
#include <vector>
#include <string>
#include <map>
#include <stdio.h>
//#include <windows.h>

using namespace std;

class ImageSourceDir : public ImageSource
{
public:

    ImageSourceDir(const char* directory, bool keepFrame = false);
	~ImageSourceDir();

	void getIplImage(int idx);
    void getIplImage();
	void getIplImage(const std::string& fileName);

	void reloadIplImage();
	int getNumImages();
	const char* getFilename(int idx=-1);
	bool setCurrentImage(int idx);
	void setRandomImage();
	int countFiles();
	int getCurrentImageIndex();
    virtual inline void reset() {  setCurrentImage(0); };

private:

	bool m_keepFrames;
	vector <string> m_fileNames;
	map <string,IplImage*> m_loadedImagesMap;
	int m_curFile;
	int m_numFiles;
	char m_DirSpec[255 + 1];
};

#endif //IMAGE_SOUCE_DIR_H
