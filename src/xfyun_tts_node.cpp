/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

#include<linux/soundcard.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<openssl/md5.h>

#include <string>
#include <iostream>
#include <fstream>

#include <alsa/asoundlib.h>

#include "std_msgs/builtin_uint16.h"

#include <boost/thread.hpp>

#include <limits.h>
  
#define MAX_SIZE (PATH_MAX+1)

using namespace std;

#define ALSA_MAX_BUF_SIZE 65535
  
using namespace std;
using std::string;
using std::fstream;

bool interrupt = true;
boost::thread* g_play_thread;
ros::Publisher speech_file_pub;

int set_pcm_play(const char* filename);

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb+");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	// printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		// printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}//合成状态synth_status取值请参阅《讯飞语音云API文档》
	// printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

void getCurrentAbsolutePath(char* current_absolute_path)
{
	if (NULL == realpath("./", current_absolute_path))
	{
		printf("***Error***");
		exit(-1);
	}
}


void tts_callback(const std_msgs::String::ConstPtr& msg)
{
	const char* text;
	int         ret                  = MSP_SUCCESS;
  	const char* session_begin_params = "engine_type = local,voice_name=xiaoyan, text_encoding = UTF8, tts_res_path = fo|/home/hcl/res/tts/xiaoyan.jet;fo|/home/hcl/res/tts/common.jet, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2"; 
	
	const int speech_file_name_size = 32;
	char speech_file_name[ speech_file_name_size + 1 ];
	// char speech_file_dir[ ] = "~/Music/voice_materials/speech/"; 
	char speech_file_dir[ ] = "/home/hcl/Music/voice_materials/speech/"; 
	char speech_file_path[ strlen( speech_file_dir ) + speech_file_name_size + strlen( ".wav" ) + 1 ];

	text = msg->data.c_str();

	/* Generate speech file name. */
	{
		unsigned char md5_value[ 16 ];
		
		MD5( (const unsigned char*)text, strlen( text ), md5_value );

		for( int i = 0; i < 16; i ++ )
		{
			sprintf( &speech_file_name[ i*2 ], "%02x", md5_value[ i ] );
		}
		speech_file_name[ speech_file_name_size ] = 0;
	}
	
	sprintf( speech_file_path, "%s%s%s", speech_file_dir, speech_file_name, ".wav" );

	printf( "%s", speech_file_path );

	/* 合成语音文件到系统临时目录 */
	ret = text_to_speech(text, speech_file_path, session_begin_params);

	if (MSP_SUCCESS != ret)
	{
		printf("text_to_speech failed, error code: %d.\n", ret);
	}
	else
	{
		std_msgs::String _speech_file;

		_speech_file.data = speech_file_path;

		speech_file_pub.publish( _speech_file );
	}
}

string nav_status = "";
boost::mutex nav_status_mutex_; 
int is_broadcasting = 0;

int main(int argc, char* argv[])
{
	int         ret                  = MSP_SUCCESS;
	const char* login_params         = "appid = 5b90a109, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动

	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);

	    MSPLogout(); //退出登录
	}
	else
	{
		g_play_thread =NULL;
		ros::init(argc, argv, "xfyun_tts_node");
		ros::NodeHandle node_handle;

		ros::Subscriber sub = node_handle.subscribe("xfyun_tts", 1, tts_callback);

		speech_file_pub = node_handle.advertise<std_msgs::String>("VoicePlay", 1 );

		ros::spin();
	}

	return 0;
}

int set_pcm_play(const char* filename)
{
	int rc;
	int ret;
	int size = 100;
	snd_pcm_t* handle; //PCI设备句柄
	snd_pcm_hw_params_t* params;//硬件信息和PCM流配置
	snd_mixer_t *mixer;
	snd_mixer_elem_t *pcm_element;
	unsigned int val;
	int dir = 0;
	snd_pcm_uframes_t frames, periodsize;
	char *buffer;
	int nread;
	interrupt = false;

	FILE *fp = fopen(filename, "rb");
	if (fp == NULL)
	{
		ROS_ERROR("No such file!");
		return 0;
	}
	else
	{
		ROS_INFO("playing......");
	}
	wave_pcm_hdr wavHeader;
	nread = fread(&wavHeader, 1, sizeof(wave_pcm_hdr), fp);

	int channels = wavHeader.channels;
	int frequency = wavHeader.samples_per_sec;
	int bit = wavHeader.bits_per_sample;
	int datablock = wavHeader.block_align;
	
	rc=snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if(rc < 0)
	{
			perror("\nopen PCM device failed:");
			exit(1);
	}
	snd_pcm_hw_params_alloca(&params); //分配params结构体
	if(rc < 0)
	{
			perror("\nsnd_pcm_hw_params_alloca:");
			exit(1);
	}
		rc=snd_pcm_hw_params_any(handle, params);//初始化params
	if(rc < 0)
	{
			perror("\nsnd_pcm_hw_params_any:");
			exit(1);
	}
	rc=snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED); //初始化访问权限
	if(rc < 0)
	{
			perror("\nsed_pcm_hw_set_access:");
			exit(1);

	}
	//采样位数
	switch(bit / 8)
	{
	case 1:
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
		break ;
	case 2:
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
		break ;
	case 3:
			snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
		break ;
		ROS_INFO("ddd");
	}
	rc = snd_pcm_hw_params_set_channels(handle, params, channels); //设置声道,1表示单声>道，2表示立体声
	if(rc<0)
	{
			perror("\nsnd_pcm_hw_params_set_channels:");
			exit(1);
	}
	val = frequency;
	rc = snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir); //设置>频率
	if(rc < 0)
	{
			perror("\nsnd_pcm_hw_params_set_rate_near:");
			exit(1);
	}
	snd_pcm_hw_params_get_buffer_size_max(params, &frames);	
	frames = frames < ALSA_MAX_BUF_SIZE ? frames : ALSA_MAX_BUF_SIZE;
	rc = snd_pcm_hw_params_set_buffer_size_near(handle, params, &frames);
	snd_pcm_hw_params_get_period_size_min(params, &periodsize, NULL);
	if(!periodsize)
	{
		periodsize = size / 2;
	}
	rc = snd_pcm_hw_params_set_period_size_near(handle, params, &periodsize, NULL);
	rc = snd_pcm_hw_params(handle, params);
	if(rc<0)
	{
		perror("\nsnd_pcm_hw_params: ");
		exit(1);
	}
	rc=snd_pcm_hw_params_get_period_size(params, &frames, &dir); /*获取周期长度*/
	snd_mixer_open(&mixer, 1);
	snd_mixer_attach(mixer, "default");
	snd_mixer_selem_register(mixer, NULL, NULL);
	snd_mixer_load(mixer);
	for(pcm_element = snd_mixer_first_elem(mixer); pcm_element; pcm_element = snd_mixer_elem_next(pcm_element))
	{
		if(snd_mixer_elem_get_type(pcm_element) == SND_MIXER_ELEM_SIMPLE && snd_mixer_selem_is_active(pcm_element))
		{
			if(!strcmp(snd_mixer_selem_get_name(pcm_element), "Master"))
			{
				snd_mixer_selem_set_playback_volume_range(pcm_element, 0, 100);
				snd_mixer_selem_set_playback_volume_all(pcm_element, (long)100);
			}
		}
	}
	if(rc<0)
	{
			perror("\nsnd_pcm_hw_params_get_period_size:");
			exit(1);
	}
	frames = 32;
	size = frames * datablock; /*4 代表数据快长度*/
	buffer =(char*)malloc(size);
	fseek(fp, 100, SEEK_SET); //定位歌曲到数据区
	while (ros::ok() && !interrupt)
	{
		memset(buffer, 0, sizeof(buffer));
		ret = fread(buffer, 1, size, fp);
		if(ret == 0)
		{
			break;
		}
		// 写音频数据到PCM设备
		while((ret = snd_pcm_writei(handle, buffer, frames))<0)
		{
			usleep(2000);
			if (ret == -EPIPE)
			{
				/* EPIPE means underrun */
				fprintf(stderr, "underrun occurred\n");
				//完成硬件参数设置，使设备准备好
				snd_pcm_prepare(handle);
			}
			else if (ret < 0)
			{
				fprintf(stderr, "error from writei: %s\n", snd_strerror(ret));
			}
		}
	}
	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	snd_mixer_close(mixer);
	free(buffer);
	fclose(fp);
	ROS_INFO("stop");

	nav_status_mutex_.lock();
	nav_status = "";
	is_broadcasting = 0;
	nav_status_mutex_.unlock();

	return 0;
}

