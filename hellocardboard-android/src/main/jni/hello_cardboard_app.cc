/*
 * Copyright 2019 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hello_cardboard_app.h"

#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <android/log.h>

#include <array>
#include <cmath>
#include <fstream>

#include "cardboard.h"
#include <math.h>
#include <android/log.h>
#include <sys/socket.h>
#include <errno.h>

# include <stdio.h>
# include <stdlib.h>
# include <errno.h>
# include <string.h>
# include <netdb.h>
# include <sys/types.h>
# include <netinet/in.h>
# include <sys/socket.h>
# include <sys/un.h>
# include <unistd.h>

#define MESSAGE_LENGTH 300
static const char *const CONTROLLER_CLICK_EVENT = "CONTROLLER_CLICK_EVENT ";
static const char *const CONTROLLER_QUIT = "CONTROLLER_QUIT ";
static const char *const CONTROLLER_MATRIX = "CONTROLLER_MATRIX ";

using namespace std;

namespace ndk_hello_cardboard {

    namespace {

// The objects are about 1 meter in radius, so the min/max target distance are
// set so that the objects are always within the room (which is about 5 meters
// across) and the reticle is always closer than any objects.
        constexpr float kMinTargetDistance = 2.5f;
        constexpr float kMaxTargetDistance = 3.5f;
        constexpr float kMinTargetHeight = 0.5f;
        constexpr float kMaxTargetHeight = kMinTargetHeight + 3.0f;

        constexpr float kDefaultFloorHeight = -1.7f;

        constexpr uint64_t kPredictionTimeWithoutVsyncNanos = 50000000;

// Angle threshold for determining whether the controller is pointing at the
// object.
        constexpr float kAngleLimit = 0.2f;

// Number of different possible targets
        constexpr int kTargetMeshCount = 3;

// Simple shaders to render .obj files without any lighting.
        constexpr const char* kObjVertexShader =
                R"glsl(
    uniform mat4 u_MVP;
    attribute vec4 a_Position;
    attribute vec2 a_UV;
    varying vec2 v_UV;

    void main() {
      v_UV = a_UV;
      gl_Position = u_MVP * a_Position;
    })glsl";

        constexpr const char* kObjFragmentShader =
                R"glsl(
    precision mediump float;

    uniform sampler2D u_Texture;
    varying vec2 v_UV;

    void main() {
      // The y coordinate of this sample's textures is reversed compared to
      // what OpenGL expects, so we invert the y coordinate.
      gl_FragColor = texture2D(u_Texture, vec2(v_UV.x, 1.0 - v_UV.y));
    })glsl";

        int currentFrame;

        int messageNumber;
        int communicationSocket; // Used to communicate from Controller to Headset

        const char *hostname = "192.168.1.39";
        struct sockaddr_in address;
        int portNumber = 5555;

        ThreadData customThreadData;

        /**
         * Receive information from a single client via an already-created thread.
         */
        class SingleClientHandler{
        public:

            bool keepLooping;

            void talkToSingleClient(int * new_socket) {

                keepLooping = true;
                do {
                    vector<char> buffer(5000);
                    int bytes = 0;
                    do {
                        bytes = recv(*new_socket, buffer.data(), buffer.size(), 0);
                    } while (bytes <= 0);
                    string message(buffer.begin(), buffer.end());
                    processClientMessage(message);
                } while (keepLooping);
            }

            /** Process the messages received from the client.
             *
             * @param recv_client_msg the message may start with "MOVECUBE", "CONTROLLER_QUIT", "MATRIX"
             * "MOVECUBE dx dy dz"
             * "CONTROLLER_QUIT"
             * "MATRIX numerical values of the client's 3d location"
             */
            void processClientMessage(string message){ //, ThreadData &data){
                if(HelloCardboardApp::startswith(message, "MOVE")){

                    string message_copy = message;
                    string delimiter = " ";

                    vector<string> result = HelloCardboardApp::string_split(message_copy, delimiter);

                    float dx = stof(result[1]);
                    float dy = stof(result[2]);
                    float dz = stof(result[3]);

                    customThreadData.cubeCurrentX += dx;
                    customThreadData.cubeCurrentY += dy;
                    customThreadData.cubeCurrentZ += dz;

                } else if(HelloCardboardApp::startswith(message, CONTROLLER_CLICK_EVENT)){
                    LOGD("arthur controller click");
                } else if(HelloCardboardApp::startswith(message, CONTROLLER_MATRIX)){
                    LOGD("arthur received matrix from controller ; uncomment this");
                    vector matrix_values = HelloCardboardApp::string_split(message, " ");
                    int rank = 1;
                    LOGD("arthur Received matrix:");
                    for(int row = 0; row < 4; row++){
                        for(int col = 0; col < 4; col++) {
                            customThreadData.controller_position_matrix.m[row][col] = std::stof(matrix_values.at(rank));
                            rank++;
                        }
                        LOGD("arthur   %f %f %f %f",
                             customThreadData.controller_position_matrix.m[row][0],
                             customThreadData.controller_position_matrix.m[row][1],
                             customThreadData.controller_position_matrix.m[row][2],
                             customThreadData.controller_position_matrix.m[row][3]);
                    }
                    LOGD("arthur Received matrix end.");

                } else if(HelloCardboardApp::startswith(message, CONTROLLER_QUIT)){
                    LOGD("arthur received message CONTROLLER_QUIT");
                    keepLooping = false;
                }

            }
        };

        /**
         * Listen to all incoming connections and create a custom thread for each client.
         */
        class AllClientsHandler{
        public:

            void receiveNewClients(){
                LOGD("arthur receiveNewClients()");

                // Create Socket
                int socket_fd = socket(AF_INET, SOCK_STREAM, 0); // creates IP based TCP socket
                if (socket_fd < 0) {
                    LOGD("arthur socket creation error");
                }

                // Bind Socket
                struct sockaddr_in server_addr;
                server_addr.sin_family = AF_INET; // sets to use IP
                server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // sets our local IP address
                server_addr.sin_port = htons(portNumber); // sets the server port number
                int status; // used to check status of c functions return values
                // using "::bind" instead of "bind" to show it is not std::bind
                status = ::bind(socket_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
                if(status < 0){
                    LOGD("arthur error binding");
                }

                while(true) {
                    LOGD("arthur receiveNewClients in thread loop");
                    // This part is moved to the thread.
                    // Listen to Socket
                    int status = listen(socket_fd, 10);
                    if (status < 0) {
                        LOGD("arthur error listening");
                    } else {
                        LOGD("arthur listening OK");
                    }

                    struct sockaddr_in client_addr;
                    socklen_t client_size = sizeof(client_addr);
                    int new_socket = accept(socket_fd, (struct sockaddr *) &client_addr, &client_size);
                    LOGD("arthur accepted connection.");

                    // New Thread
                    SingleClientHandler handler;
                    std::thread custom_thread(&SingleClientHandler::talkToSingleClient, &handler, &new_socket);
                    custom_thread.detach();
                }
                LOGD("arthur THREAD END");
            }
        };

        AllClientsHandler threadWorker;

    }  // anonymous namespace

    HelloCardboardApp::HelloCardboardApp(JavaVM* vm, jobject obj,
                                         jobject asset_mgr_obj)
            : head_tracker_(nullptr),
              lens_distortion_(nullptr),
              distortion_renderer_(nullptr),
              screen_params_changed_(false),
              device_params_changed_(false),
              screen_width_(0),
              screen_height_(0),
              depthRenderBuffer_(0),
              framebuffer_(0),
              texture_(0),
              obj_program_(0),
              obj_position_param_(0),
              obj_uv_param_(0),
              obj_modelview_projection_param_(0),
              target_object_meshes_(kTargetMeshCount),
              target_object_not_selected_textures_(kTargetMeshCount),
              target_object_selected_textures_(kTargetMeshCount),
              cur_target_object_(RandomUniformInt(kTargetMeshCount)),
              initMenuCoordinatesArray_(0),
              texture_array_(10){

        __android_log_print(ANDROID_LOG_VERBOSE, "arthur_3d", "arthur HelloCardboardApp start %d", 0);

        JNIEnv* env;
        vm->GetEnv((void**)&env, JNI_VERSION_1_6);
        java_asset_mgr_ = env->NewGlobalRef(asset_mgr_obj);
        asset_mgr_ = AAssetManager_fromJava(env, asset_mgr_obj);

        Cardboard_initializeAndroid(vm, obj);
        head_tracker_ = CardboardHeadTracker_create();

        // A single cube moves across the room and must be caught by the player.
        isCubeMoving = false;

        customThreadData.cubeCurrentX = 0;
        customThreadData.cubeCurrentY = 1;
        customThreadData.cubeCurrentZ = 0;

        roomWidth = 0;
        roomHeight = 0;

        // Initialize random number generator
        srand(time(0));

        currentFrame = 0;
        messageNumber = 0;

        isHeadset = false;
        isController = false;

        viewer_position_x = 0;
        viewer_position_y = -1;
        viewer_position_z = 0;

        setupInitMenuCoordinates();
    }

    HelloCardboardApp::~HelloCardboardApp() {
        CardboardHeadTracker_destroy(head_tracker_);
        CardboardLensDistortion_destroy(lens_distortion_);
        CardboardDistortionRenderer_destroy(distortion_renderer_);
    }


    void HelloCardboardApp::OnSurfaceCreated(JNIEnv* env) {
        const int obj_vertex_shader =
                LoadGLShader(GL_VERTEX_SHADER, kObjVertexShader);
        const int obj_fragment_shader =
                LoadGLShader(GL_FRAGMENT_SHADER, kObjFragmentShader);

        obj_program_ = glCreateProgram();
        glAttachShader(obj_program_, obj_vertex_shader);
        glAttachShader(obj_program_, obj_fragment_shader);
        glLinkProgram(obj_program_);
        glUseProgram(obj_program_);

        CHECKGLERROR("Obj program");

        obj_position_param_ = glGetAttribLocation(obj_program_, "a_Position");
        obj_uv_param_ = glGetAttribLocation(obj_program_, "a_UV");
        obj_modelview_projection_param_ = glGetUniformLocation(obj_program_, "u_MVP");

        CHECKGLERROR("Obj program params");

        HELLOCARDBOARD_CHECK(room_.Initialize(obj_position_param_, obj_uv_param_,
                                              "CubeRoom.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(cube_.Initialize(obj_position_param_, obj_uv_param_,
                                              "arthurCube.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(controller_.Initialize(obj_position_param_, obj_uv_param_,
                                                    "arthurCube.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(
                room_tex_.Initialize(env, java_asset_mgr_, "CubeRoom_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(
                cube_tex_.Initialize(env, java_asset_mgr_, "arthur_cube.png"));
        HELLOCARDBOARD_CHECK(
                cube_tex_selected_.Initialize(env, java_asset_mgr_, "arthur_cube_selected.png"));
        HELLOCARDBOARD_CHECK(
                controller_tex_.Initialize(env, java_asset_mgr_, "arthur_cube_selected.png"));
        HELLOCARDBOARD_CHECK(target_object_meshes_[0].Initialize(
                obj_position_param_, obj_uv_param_, "Icosahedron.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[0].Initialize(
                env, java_asset_mgr_, "Icosahedron_Blue_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(target_object_selected_textures_[0].Initialize(
                env, java_asset_mgr_, "Icosahedron_Pink_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(target_object_meshes_[1].Initialize(
                obj_position_param_, obj_uv_param_, "QuadSphere.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[1].Initialize(
                env, java_asset_mgr_, "QuadSphere_Blue_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(target_object_selected_textures_[1].Initialize(
                env, java_asset_mgr_, "QuadSphere_Pink_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(target_object_meshes_[2].Initialize(
                obj_position_param_, obj_uv_param_, "TriSphere.obj", asset_mgr_));
        HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[2].Initialize(
                env, java_asset_mgr_, "TriSphere_Blue_BakedDiffuse.png"));
        HELLOCARDBOARD_CHECK(target_object_selected_textures_[2].Initialize(
                env, java_asset_mgr_, "TriSphere_Pink_BakedDiffuse.png"));

        HELLOCARDBOARD_CHECK(texture_array_[0].Initialize(
                env, java_asset_mgr_, "menu_headset.png"));
        HELLOCARDBOARD_CHECK(texture_array_[1].Initialize(
                env, java_asset_mgr_, "menu_controller.png"));
        HELLOCARDBOARD_CHECK(texture_array_[2].Initialize(
                env, java_asset_mgr_, "controller_tex.png"));

        // Target object first appears directly in front of user.
        target_position_matrix = GetTranslationMatrix({0.0f, 1.5f, -kMinTargetDistance});
        cube_position_matrix = GetTranslationMatrix({0.0f, 1.5f, 0.0f});
        customThreadData.controller_position_matrix = GetTranslationMatrix({0.5f, 2.0f, 0.0f});
        roomWidth = 10;
        roomHeight = 7;

        CHECKGLERROR("OnSurfaceCreated");
    }

    void HelloCardboardApp::SetScreenParams(int width, int height) {
        screen_width_ = width;
        screen_height_ = height;
        screen_params_changed_ = true;
    }

    void HelloCardboardApp::OnDrawFrame() {
        if(isController) {
            // Used only for the controller
            OnDrawFrameController();
        }
        else{
            // Used both for headset and before selection
            OnDrawFrameHeadset();
        }
    }

    void HelloCardboardApp::OnDrawFrameController() {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0,0,255,255);

        // Update Head Pose.
        head_view_ = GetPose();

        // Send the controller's matrix to the headset
        if(1000*(messageNumber/1000) == messageNumber) {
            std::string textMatrix4x4 = CONTROLLER_MATRIX + head_view_.toString();
            char *message = new char[textMatrix4x4.length() + 1];

            strcpy(message, textMatrix4x4.c_str());

            LOGD("arthur formatting message...");
            vector<string> values = HelloCardboardApp::string_split(textMatrix4x4, " ");

            LOGD("arthur formatting message... 2");
            std::string formattedMessage = "";
            for (int i = 0; i < 17; i++) { // Write "CONTROLLER_MATRIX ", then all 16 values.
                LOGD("arthur formatting message... 3 <%s>", formattedMessage.c_str());
                formattedMessage += values[i] + " ";
            }

            LOGD("arthur formatted message is <%s>", formattedMessage.c_str());

            //        LOGD("arthur Controller sending matrix to headset: %s", message);
            //        LOGD("arthur size of message sent: %d", strlen(message));
            sendMessageToHeadset(const_cast<char *>(formattedMessage.c_str()));
        }

//        LOGD("arthur OnDrawFrameController DONE");
    }

    void HelloCardboardApp::OnDrawFrameHeadset() {
        if (UpdateDeviceParams()) {

            // Update Head Pose.
            head_view_ = GetPose();

            // Incorporate the floor height into the head_view
            head_view_ = head_view_ * GetTranslationMatrix({viewer_position_x, viewer_position_y, viewer_position_z});

            // Bind buffer
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

            glEnable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);
            glDisable(GL_SCISSOR_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


            // This part must be taken care of in a timer.
//            if (isCubeMoving) {
//
//                customThreadData.cubeCurrentX += cubeVx;
//                customThreadData.cubeCurrentY += cubeVy;
//                customThreadData.cubeCurrentZ += cubeVz;
//
//                boundSpeed();
//            }
            cube_position_matrix = GetTranslationMatrix({customThreadData.cubeCurrentX, customThreadData.cubeCurrentY, customThreadData.cubeCurrentZ});


            // Draw eyes views
            for (int eye = 0; eye < 2; ++eye) {
                glViewport(eye == kLeft ? 0 : screen_width_ / 2, 0, screen_width_ / 2,
                           screen_height_);

                Matrix4x4 eye_matrix = GetMatrixFromGlArray(eye_matrices_[eye]);
                Matrix4x4 eye_view = eye_matrix * head_view_;

                Matrix4x4 eye_projection_matrix = GetMatrixFromGlArray(projection_matrices_[eye]);
                Matrix4x4 modelview_target = eye_view * target_position_matrix;
                target_projection_matrix_ = eye_projection_matrix * modelview_target;
                roomProjectionMatrix_ = eye_projection_matrix * eye_view;
                cube_projection_matrix_ = eye_projection_matrix * eye_view * cube_position_matrix;
                controller_projection_matrix_ = eye_projection_matrix * eye_view * customThreadData.controller_position_matrix;

                // Draw room and target
                DrawWorld(eye_projection_matrix * eye_view);
            }

            // Render
            CardboardDistortionRenderer_renderEyeToDisplay(
                    distortion_renderer_, /* target_display = */ 0, /* x = */ 0, /* y = */ 0,
                    screen_width_, screen_height_, &left_eye_texture_description_,
                    &right_eye_texture_description_);

            CHECKGLERROR("onDrawFrame");
        }
    }

    void HelloCardboardApp::OnTriggerEvent(jfloat x, jfloat y) {
        LOGD("arthur on trigger");
        if (isHeadset) {
            OnTriggerEventHeadset();
        } else if (isController) {
            OnTriggerEventController(x, y);
        } else {
            DefineHeadsetOrController();
        }
    }

    void HelloCardboardApp::OnTriggerEventHeadset() {
        if (IsPointingAtTarget()) {
            HideTarget();
        }
        if (IsPointingAtTarget(cube_position_matrix)) {
            toggleCubeMovement();
        }
    }

    void HelloCardboardApp::OnTriggerEventController(jfloat x, jfloat y) {

        LOGD("arthur trigger event at (%f, %f) on controller",
             x, y);

        int xInt = x;
        int yInt = y;

        string s = CONTROLLER_CLICK_EVENT + std::to_string(xInt) + " " + std::to_string(yInt);
        char message[s.length()+1];
        strcpy(message, s.c_str());
        sendMessageToHeadset(message);
    }

    void HelloCardboardApp::sendMessageToHeadset(char message[]) const {
        int n;

        char buffer[MESSAGE_LENGTH] = "";

        std::fill(std::begin(buffer), std::end(buffer), '\0');
        strcpy(buffer, message);
        n = write(communicationSocket, buffer, MESSAGE_LENGTH);
        if (n < 0) {
            LOGD("arthur n<0");
        } else {
            LOGD("arthur n>=0");
        }
        LOGD("arthur after sending message <%s>", message);
    }

    void HelloCardboardApp::closeSockets(){
        LOGD("arthur closing socket...");
        close(communicationSocket);
        LOGD("arthur socket closed");
    }

    void HelloCardboardApp::DefineHeadsetOrController() {
        LOGD("arthur define headset or controller");

        int rank = 0;
        for (Matrix4x4 mat: initMenuCoordinatesArray_) {
            if (IsPointingAtTarget(mat)) {
                if (getInitMenuValue(rank) == 0) {
                    isHeadset = true;
                    // Initial player position/coordinates (when the phone is the headset)
                    viewer_position_x = -2.0f;
                    viewer_position_y = kDefaultFloorHeight;
                    viewer_position_z = -2.0f;
                    // Place the player high above the room for testing purposes
//                    viewer_position_x = 0.0f; //-2.0f;
//                    viewer_position_y = -10.0f;// kDefaultFloorHeight;
//                    viewer_position_z = 0.0f; //-2.0f;
                    LOGD("arthur setting up HEADSET");
                    setupServer();
                    LOGD("arthur setting up HEADSET DONE");
                } else {
                    isController = true;
                    LOGD("arthur setting up HEADSET");
                    setupServer();
                    LOGD("arthur setting up client DONE");
                }
            }
            rank++;
        }
    }

    void HelloCardboardApp::OnPause() { CardboardHeadTracker_pause(head_tracker_); }

    void HelloCardboardApp::OnResume() {
        CardboardHeadTracker_resume(head_tracker_);

        // Parameters may have changed.
        device_params_changed_ = true;

        // Check for device parameters existence in external storage. If they're
        // missing, we must scan a Cardboard QR code and save the obtained parameters.
        uint8_t* buffer;
        int size;
        CardboardQrCode_getSavedDeviceParams(&buffer, &size);
        if (size == 0) {
            SwitchViewer();
        }
        CardboardQrCode_destroy(buffer);
    }

    void HelloCardboardApp::SwitchViewer() {
        CardboardQrCode_scanQrCodeAndSaveDeviceParams();
    }

    bool HelloCardboardApp::UpdateDeviceParams() {
        // Checks if screen or device parameters changed
        if (!screen_params_changed_ && !device_params_changed_) {
            return true;
        }

        // Get saved device parameters
        uint8_t* buffer;
        int size;
        CardboardQrCode_getSavedDeviceParams(&buffer, &size);

        // If there are no parameters saved yet, returns false.
        if (size == 0) {
            return false;
        }

        CardboardLensDistortion_destroy(lens_distortion_);
        lens_distortion_ = CardboardLensDistortion_create(buffer, size, screen_width_,
                                                          screen_height_);

        CardboardQrCode_destroy(buffer);

        GlSetup();

        CardboardDistortionRenderer_destroy(distortion_renderer_);
        const CardboardOpenGlEsDistortionRendererConfig config{kGlTexture2D};
        distortion_renderer_ = CardboardOpenGlEs2DistortionRenderer_create(&config);

        CardboardMesh left_mesh;
        CardboardMesh right_mesh;
        CardboardLensDistortion_getDistortionMesh(lens_distortion_, kLeft,
                                                  &left_mesh);
        CardboardLensDistortion_getDistortionMesh(lens_distortion_, kRight,
                                                  &right_mesh);

        CardboardDistortionRenderer_setMesh(distortion_renderer_, &left_mesh, kLeft);
        CardboardDistortionRenderer_setMesh(distortion_renderer_, &right_mesh,
                                            kRight);

        // Get eye matrices
        CardboardLensDistortion_getEyeFromHeadMatrix(lens_distortion_, kLeft,
                                                     eye_matrices_[0]);
        CardboardLensDistortion_getEyeFromHeadMatrix(lens_distortion_, kRight,
                                                     eye_matrices_[1]);
        CardboardLensDistortion_getProjectionMatrix(lens_distortion_, kLeft, kZNear,
                                                    kZFar, projection_matrices_[0]);
        CardboardLensDistortion_getProjectionMatrix(lens_distortion_, kRight, kZNear,
                                                    kZFar, projection_matrices_[1]);

        screen_params_changed_ = false;
        device_params_changed_ = false;

        CHECKGLERROR("UpdateDeviceParams");

        return true;
    }

    void HelloCardboardApp::GlSetup() {
        LOGD("GL SETUP");

        if (framebuffer_ != 0) {
            GlTeardown();
        }

        // Create render texture.
        glGenTextures(1, &texture_);
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, screen_width_, screen_height_, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, 0);

        left_eye_texture_description_.texture = texture_;
        left_eye_texture_description_.left_u = 0;
        left_eye_texture_description_.right_u = 0.5;
        left_eye_texture_description_.top_v = 1;
        left_eye_texture_description_.bottom_v = 0;

        right_eye_texture_description_.texture = texture_;
        right_eye_texture_description_.left_u = 0.5;
        right_eye_texture_description_.right_u = 1;
        right_eye_texture_description_.top_v = 1;
        right_eye_texture_description_.bottom_v = 0;

        // Generate depth buffer to perform depth test.
        glGenRenderbuffers(1, &depthRenderBuffer_);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, screen_width_,
                              screen_height_);
        CHECKGLERROR("Create Render buffer");

        // Create render target.
        glGenFramebuffers(1, &framebuffer_);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                               texture_, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, depthRenderBuffer_);

        CHECKGLERROR("GlSetup");
    }

    void HelloCardboardApp::GlTeardown() {
        if (framebuffer_ == 0) {
            return;
        }
        glDeleteRenderbuffers(1, &depthRenderBuffer_);
        depthRenderBuffer_ = 0;
        glDeleteFramebuffers(1, &framebuffer_);
        framebuffer_ = 0;
        glDeleteTextures(1, &texture_);
        texture_ = 0;

        CHECKGLERROR("GlTeardown");
    }

    Matrix4x4 HelloCardboardApp::GetPose() {
        std::array<float, 4> out_orientation;
        std::array<float, 3> out_position;
        CardboardHeadTracker_getPose(
                head_tracker_, GetBootTimeNano() + kPredictionTimeWithoutVsyncNanos,
                kLandscapeLeft, &out_position[0], &out_orientation[0]);
        return GetTranslationMatrix(out_position) *
               Quatf::FromXYZW(&out_orientation[0]).ToMatrix();
    }

    void HelloCardboardApp::DrawWorld(Matrix4x4 projectionMatrix) {
        currentFrame++;

        if (isHeadset) {
            DrawRoom();
            DrawCube();
            DrawController();
//            DrawTarget();
        } else if (isController) {
            // Draw buttons on the controller screen; the virtual controller will be drawn in the headset.
        } else {
            // Draw the init menu
            DrawInitMenu(projectionMatrix);
        }
    }

    void HelloCardboardApp::setupInitMenuCoordinates(){

        for(int row= -nbMenuRows / 2; row < nbMenuRows / 2; row++) {

            for (int rank = 0; rank < nbMenuCubesPerRow; rank++) {
                double angle = rank * 2 * M_PI / nbMenuCubesPerRow;
                double radius = 3.0f;
                float cubeX = radius * std::cos(angle);
                float cubeY = 2 * (row + .5);
                float cubeZ = radius * std::sin(angle);

                initMenuCoordinatesArray_.push_back(GetTranslationMatrix({cubeX, cubeY, cubeZ}));
            }
        }
    }

    // Draw the menu cubes, with the appropriate textures
    void HelloCardboardApp::DrawInitMenu(Matrix4x4 projectionMatrix) {

        for(int i_cube = 0; i_cube < nbMenuRows * nbMenuCubesPerRow; i_cube++) {
            if(i_cube < initMenuCoordinatesArray_.size()) {
                Matrix4x4 cubeProjectionMatrix = projectionMatrix * initMenuCoordinatesArray_.at(i_cube);
                int i_tex = getInitMenuValue(i_cube);
                DrawCube(cubeProjectionMatrix, i_tex);
            }
        }
    }

    void HelloCardboardApp::DrawTarget() {
        glUseProgram(obj_program_);

        std::array<float, 16> target_array = target_projection_matrix_.ToGlArray();
        glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                           target_array.data());

        if (IsPointingAtTarget()) {
            target_object_selected_textures_[cur_target_object_].Bind();
        } else {
            target_object_not_selected_textures_[cur_target_object_].Bind();
        }
        target_object_meshes_[cur_target_object_].Draw();

        CHECKGLERROR("DrawTarget");
    }

    void HelloCardboardApp::DrawRoom() {
        glUseProgram(obj_program_);

        std::array<float, 16> room_array = roomProjectionMatrix_.ToGlArray();
        glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                           room_array.data());

        room_tex_.Bind();
        room_.Draw();

        CHECKGLERROR("DrawRoom");
    }

    void HelloCardboardApp::DrawCube() {
        DrawCube(cube_projection_matrix_);
    }
    void HelloCardboardApp::DrawController() {
        DrawController(controller_projection_matrix_);
    }

    void HelloCardboardApp::DrawCube(Matrix4x4 projection_matrix_) {
        DrawCube(projection_matrix_, -1);
    }
    void HelloCardboardApp::DrawController(Matrix4x4 projection_matrix_) {
        DrawController(projection_matrix_, -1);
    }

    /**
     *
     * @param projection_matrix_
     * @param texture_id if -1: texture is defined by the DrawCube function; other: index of texture in array
     */
    void HelloCardboardApp::DrawCube(Matrix4x4 projection_matrix_, int texture_id) {
        glUseProgram(obj_program_);

        std::array<float, 16> cube_array = projection_matrix_.ToGlArray();
        glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                           cube_array.data());
        if(texture_id==-1) {
            if (IsPointingAtTarget(cube_position_matrix)) {
                cube_tex_selected_.Bind();
            } else {
                cube_tex_.Bind();
            }
        } else{
            // use the texture_id-th texture in the array
            if(texture_id < texture_array_.size()) {
                texture_array_.at(texture_id).Bind();
            }
            else{
                // Use the default texture
                cube_tex_.Bind();
            }
        }
        cube_.Draw();

        CHECKGLERROR("DrawCube");
    }


    /**
     *
     * @param projection_matrix_
     * @param texture_id if -1: texture is defined by the DrawCube function; other: index of texture in array
     */
    void HelloCardboardApp::DrawController(Matrix4x4 projection_matrix_, int texture_id) {
        glUseProgram(obj_program_);

        std::array<float, 16> controller_array = projection_matrix_.ToGlArray();
        glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                           controller_array.data());
        controller_tex_.Bind();

        controller_.Draw();

        CHECKGLERROR("DrawController");
    }

    void HelloCardboardApp::HideTarget() {
        cur_target_object_ = RandomUniformInt(kTargetMeshCount);

        float angle = RandomUniformFloat(-M_PI, M_PI);
        float distance = RandomUniformFloat(kMinTargetDistance, kMaxTargetDistance);
        float height = RandomUniformFloat(kMinTargetHeight, kMaxTargetHeight);
        std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                                std::sin(angle) * distance};

        target_position_matrix = GetTranslationMatrix(target_position);
    }

    bool HelloCardboardApp::IsPointingAtTarget() {
        // Compute vectors pointing towards the reticle and towards the target object
        // in head space.
        Matrix4x4 head_from_target = head_view_ * target_position_matrix;

        const std::array<float, 4> unit_quaternion = {0.f, 0.f, 0.f, 1.f};
        const std::array<float, 4> point_vector = {0.f, 0.f, -1.f, 0.f};
        const std::array<float, 4> target_vector = head_from_target * unit_quaternion;

        float angle = AngleBetweenVectors(point_vector, target_vector);
        return angle < kAngleLimit;
    }


    bool HelloCardboardApp::IsPointingAtTarget(Matrix4x4 param) {
        // Compute vectors pointing towards the reticle and towards the target object
        // in head space.
        Matrix4x4 head_from_target = head_view_ * param;

        const std::array<float, 4> unit_quaternion = {0.f, 0.f, 0.f, 1.f};
        const std::array<float, 4> point_vector = {0.f, 0.f, -1.f, 0.f};
        const std::array<float, 4> target_vector = head_from_target * unit_quaternion;

        float angle = AngleBetweenVectors(point_vector, target_vector);
        return angle < kAngleLimit;
    }

    // When the cube is at the borders, flip the speed;
    void HelloCardboardApp::boundSpeed() {
        float margin = 0.2;

        if (customThreadData.cubeCurrentY < -0 && cubeVy < 0) {
            customThreadData.cubeCurrentY = margin;
            cubeVy = -cubeVy;
        }
        if (customThreadData.cubeCurrentY > roomHeight && cubeVy > 0) {
            customThreadData.cubeCurrentY = roomHeight - margin;
            cubeVy = -cubeVy;
        }

        if (customThreadData.cubeCurrentX < -roomWidth / 2 && cubeVy < 0) {
            customThreadData.cubeCurrentX = -roomWidth / 2 + margin;
            cubeVx = -cubeVx;
        }
        if (customThreadData.cubeCurrentX > roomWidth / 2 && cubeVx > 0) {
            customThreadData.cubeCurrentX = roomWidth / 2 - margin;
            cubeVx = -cubeVx;
        }

        if (customThreadData.cubeCurrentZ < -roomWidth / 2 && cubeVz < 0) {
            customThreadData.cubeCurrentZ = -roomWidth / 2 + margin;
            cubeVz = -cubeVz;
        }
        if (customThreadData.cubeCurrentZ > roomWidth / 2 && cubeVz > 0) {
            customThreadData.cubeCurrentZ = roomWidth / 2 - margin;
            cubeVz = -cubeVz;
        }
    }

    void HelloCardboardApp::toggleCubeMovement() {
        isCubeMoving = !isCubeMoving;
        if (isCubeMoving) {
            // Compute random speed
            computeRandomSpeed();
        }
    }

    float HelloCardboardApp::random(float a, float b) {
        int random_int = rand();
        float res_01 = (float) random_int / RAND_MAX;
        float res_ab = res_01 * (b - a) + a;
        return res_ab;
    }

    void HelloCardboardApp::computeRandomSpeed() {
        float v_max = 0.05;
        cubeVx = random(-v_max, v_max);
        cubeVy = random(-v_max, v_max);
        cubeVz = random(-v_max, v_max);

    }

    /** Determines what menuItem is associated with a given cube of the init menu.
     *
     * @param cube_index the rank of the menu cube
     * @return the menu index
     */
    int HelloCardboardApp::getInitMenuValue(int cube_index) {
        return cube_index % 2;
    }

    /** Setup the socket that allows the controller to talk to the headset.
     *
     */
    void HelloCardboardApp::setupServer() {
        if(isHeadset){
            setupServerForHeadset();
        }else if(isController) {
            setupServerForController();
        }
    }


    /** The headset is the server-side of the TCP communication.
     *
     */
    void HelloCardboardApp::setupServerForHeadset() const {

        std::thread my_thread(&AllClientsHandler::receiveNewClients, &threadWorker);
        my_thread.detach();
    }




    vector<string> HelloCardboardApp::string_split(string &message, const string &delimiter) {
        __ndk1::vector<string> result;
        size_t pos = 0;
        string token;

        while((pos = message.find(delimiter)) != std::string::npos){
            token = message.substr(0, pos);
            result.push_back(token);
            message.erase(0, pos + delimiter.length());
        }
        result.push_back(message);
        return result;
    }

    bool HelloCardboardApp::startswith(string s, string sub) {
        return (s.substr(0, sub.length()) == sub);
    }

    /** The controller is the client-side of the TCP communication.
     *
     */
    void HelloCardboardApp::setupServerForController() const {
        LOGD("arthur setting up client isController");


        struct hostent *server;
        struct sockaddr_in serv_addr;

        communicationSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (communicationSocket < 0) {
            LOGD("arthur ERROR opening socket");
        }
        LOGD("arthur after socket creation");
        server = gethostbyname(hostname);
        LOGD("arthur after gethostbyname: server is %s", server);
        if (server == NULL) {
            LOGD("arthur gethostbyname: server is NULL");
            fprintf(stderr, "ERROR, no such host\n");
            exit(0);
        }
        LOGD("arthur gethostbyname: server is not NULL");
        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char *) server->h_addr,
              (char *) &serv_addr.sin_addr.s_addr,
              server->h_length);
        serv_addr.sin_port = htons(portNumber);
        LOGD("ARTHUR testing connect...");
        if (connect(communicationSocket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
            LOGD("arthur connecting PAS OK");
        } else {
            LOGD("arthur connecting OK");
        }
    }
}  // namespace ndk_hello_cardboard