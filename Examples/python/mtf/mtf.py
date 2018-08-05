import pyMTF2
import os

class VideoCapture():
    def __init__(self, cap_src='', cap_args=''):
        if cap_src:
            try:
                mtf_args = 'img_source u'
            except ValueError:
                if os.path.isdir(cap_src):
                    mtf_args = 'img_source j seq_path {:s} seq_fmt jpg'.format(cap_src)
                elif os.path.isfile(cap_src):
                    seq_fmt = os.path.splitext(os.path.basename(cap_src))[1][1:]
                    mtf_args = 'img_source m seq_path {:s} seq_fmt {}'.format(cap_src, seq_fmt)
                else:
                    raise IOError('Invalid capture source: {}'.format(cap_src))
        else:
            mtf_args = ''

        if cap_args:
            if mtf_args:
                mtf_args = '{} {}'.format(mtf_args, cap_args)
            else:
                mtf_args = cap_args
        if not pyMTF2.init(mtf_args):
            raise SystemError('MTF input pipeline creation was unsuccessful')
        else:
            print('MTF input pipeline created successfully')

    def get(self, info_id):
        src_img = pyMTF2.getFrame()
        if src_img is None:
            print('Frame extraction was unsuccessful')
            return None
        if info_id == 3 or info_id == 'width':
            return src_img.shape[1]
        elif info_id == 4 or info_id == 'height':
            return src_img.shape[0]
        else:
            print('iInvalid info ID: {}'.format(info_id))
            return None

    def read(self):
        src_img = pyMTF2.getFrame()
        if src_img is None:
            print('Frame extraction was unsuccessful')
            return 0, None
        return 1, src_img

    def release(self):
        pyMTF2.quit()

