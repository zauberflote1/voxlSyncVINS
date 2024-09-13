/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "all_interfaces.h"
#include "interface_manager.h"

#define THREAD_INTERVAL_MS 1000

static void* ThreadManageInterfaces(void *pData);

static bool pipeExists(const char *pipeName);

// -----------------------------------------------------------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------------------------------------------------------
InterfaceManager::InterfaceManager(ros::NodeHandle nh, ros::NodeHandle nhp)
{

    m_threadData.manager = this;
    m_threadData.running = false;
    m_threadData.nh = nh;
    m_threadData.nhp = nhp;

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function opens the camera and starts sending the capture requests
// -----------------------------------------------------------------------------------------------------------------------------
void InterfaceManager::Start()
{

    m_threadData.running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_create(&m_thread, &attr, ThreadManageInterfaces, &m_threadData);
    pthread_attr_destroy(&attr);

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function informs the per camera manager to stop the camera and stop sending any more requests
// -----------------------------------------------------------------------------------------------------------------------------
void InterfaceManager::Stop()
{
    m_threadData.running = false;

    pthread_join(m_thread, NULL);
}

typedef struct InterfaceListNode {

	GenericInterface *interface;
	char name[64];
    struct InterfaceListNode *next;

} InterfaceListNode;

static bool listContainsPipe(InterfaceListNode *head, char *name){

    for(InterfaceListNode *cur = head->next; cur != NULL; cur = cur->next){

    	if(!strcmp(name, cur->name)){

    		return true;
    	}
    }

    return false;
}

static int findPipes(InterfaceListNode *head, ros::NodeHandle nh, ros::NodeHandle nhp){

	InterfaceListNode *tail;
    for(tail = head; tail->next != NULL; tail = tail->next);

	FILE *fp = popen("voxl-list-pipes --mode-types", "r");
	if (!fp) {
		fprintf(stderr, "Error opening list-pipes command\n");
		return -1;
	}

	InterfaceType curType = INT_NONE;
	char buf[64];
	while(fgets(buf, 64, fp) != NULL){
		//printf("%s", buf);

		//Empty line, about to recieve new type
		if(strlen(buf) == 1){
			curType = INT_NONE;
			continue;
		}

		if(curType == INT_NONE){

			// This is a pipe name not a type, something has gone wrong
			if(buf[0] == '\t'){
				// fprintf(stderr, "Parse error while reading voxl-list-pipes (looking for pipe), exiting\n");
				// pclose(fp);
				// return -1;
				continue;
			}

			if(!strncmp(buf, "camera_image_metadata_t", strlen("camera_image_metadata_t"))){
				//printf("Processing Type: %s\n", buf);
				curType = INT_CAMERA;
			// } else if(!strncmp(buf, "imu_data_t", strlen("imu_data_t"))){
			// 	//printf("Processing Type: %s\n", buf);
			// 	curType = INT_IMU;
			// } else if(!strncmp(buf, "vio_data_t", strlen("vio_data_t"))){
			// 	//printf("Processing Type: %s\n", buf);
			// 	curType = INT_VIO;
			// } else if(!strncmp(buf, "point_cloud_metadata_t", strlen("point_cloud_metadata_t"))){
			// 	//printf("Processing Type: %s\n", buf);
			// 	curType = INT_PC;
            // } else if(!strncmp(buf, "ai_detection_t", strlen("ai_detection_t"))){
			// 	//printf("Processing Type: %s\n", buf);
			// 	curType = INT_AI;
			// } else {
			// 	curType = INT_NOT_SUPPORTED;
			}

		} else if(curType == INT_NOT_SUPPORTED){//This pipe has a type that we don't publish to ros, ignore the available pipes

			continue;

		} else { //Getting a pipe name for a known interface type

			// This is a type not a pipe name, something has gone wrong
			if(buf[0] != '\t'){
				// fprintf(stderr, "Parse error while reading voxl-list-pipes (looking for pipe), exiting\n");
				// pclose(fp);
				// return -1;
				continue;
			}

			char *name = &buf[1];

			name[strlen(name) - 1] = 0;//Remove newline

			if(listContainsPipe(head, name)){//This interface is already open
				continue;
			}

			printf("Found new interface: %s\n", name);

			InterfaceListNode *newNode = (InterfaceListNode *)malloc(sizeof(InterfaceListNode));
			if(newNode == NULL){
				fprintf(stderr, "Error mallocing in findPipes, Exiting\n");
				pclose(fp);
				return -1;
			}

			strcpy(newNode->name, name);
			newNode->next = NULL;

			try {
				switch(curType) {

					case INT_CAMERA:
					case INT_STEREO:
						if(strstr(newNode->name, "stereo")){
							newNode->interface = new StereoInterface(nh, nhp, newNode->name);
						} else {
							newNode->interface = new CameraInterface(nh, nhp, newNode->name);
						}
						break;

					// case INT_IMU:
					// 	newNode->interface = new IMUInterface(nh, nhp, newNode->name);
					// 	break;

					// case INT_VIO:
					// 	newNode->interface = new VIOInterface(nh, nhp, newNode->name);
					// 	break;

					// case INT_PC:
					// 	newNode->interface = new PointCloudInterface(nh, nhp, newNode->name);
					// 	break;

                    // case INT_AI:
					// 	newNode->interface = new AiDetectionInterface(nh, nhp, newNode->name);
					// 	break;

					default: //Should never get here
						// printf("Reached impossible line of code: %s %d\n", __FUNCTION__, __LINE__);
						// exit(-1);
						break;
				}

				tail->next = newNode;
				tail = newNode;
				newNode->interface->AdvertiseTopics();

			} catch (int i){

				fprintf(stderr, "Failed to open pipe: %s\n", newNode->name);
				free(newNode);

			}

		}


	}

	//fprintf(stderr, "done reading, closing pipe\n");
	pclose(fp);

	return 0;

}

static void* ThreadManageInterfaces(void *pData){

    ThreadData*        pThreadData = (ThreadData*)pData;
    InterfaceListNode  head        = {NULL, "", NULL};

    while(pThreadData->running){

    	if(findPipes(&head, pThreadData->nh, pThreadData->nhp)){
    		break;
    	}

        for(InterfaceListNode *i = head.next; i != NULL; i = i->next){

            GenericInterface *interface = i->interface;

            if(interface->GetState() == ST_READY && pipeExists(interface->GetPipeName())){
                interface->AdvertiseTopics();
                printf("Found pipe for interface: %s, now advertising\n", interface->GetPipeName());
            }

            if(interface->GetState() == ST_RUNNING && interface->GetNumClients() == 0){
                interface->StopPublishing();
                if(interface->GetState() == ST_AD){
                    printf("Interface %s ceasing to publish\n", interface->GetPipeName());
                }
                continue;
            }

            if(interface->GetState() == ST_AD && !pipeExists(interface->GetPipeName())){
                interface->StopAdvertising();
                interface->SetState(ST_READY);
                printf("Interface: %s's data pipe disconnected, closing until it returns\n", interface->GetPipeName());
                continue;
            }

            if(interface->GetState() == ST_AD && interface->GetNumClients() > 0){
                interface->StartPublishing();
                if(interface->GetState() == ST_RUNNING){
                    printf("Interface %s now publishing\n", interface->GetPipeName());
                }
                continue;
            }
        }

        usleep(THREAD_INTERVAL_MS * 1000);
    }

    //Clean the list
    for(InterfaceListNode *i = head.next; i != NULL; i = head.next){

        GenericInterface *interface = i->interface;

        if(interface->GetState() == ST_RUNNING){
            interface->StopPublishing();
        }

        if(interface->GetState() == ST_AD){
            interface->StopAdvertising();
        }

        head.next = i->next;

        delete i->interface;
        free(i);
    }

    return NULL;

}

static bool pipeExists(const char *pipeName){

    char fullPath[MODAL_PIPE_MAX_PATH_LEN];
    pipe_expand_location_string((char *)pipeName, fullPath);
    strcat(fullPath, "request");

    //return access(fullPath, F_OK) == 0;
    if (access(fullPath, F_OK) == 0){
    	return true;
    } else {
    	return false;
    }

}
