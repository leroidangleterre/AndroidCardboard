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

#ifndef HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_
#define HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_

#include <android/asset_manager.h>
#include <jni.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <GLES2/gl2.h>
#include <linux/in.h>
#include "cardboard.h"
#include "util.h"


using namespace std;

namespace ndk_hello_cardboard {


    // The information that will be shared by the commmunication thread and the main thread.
    struct ThreadData {

        float cubeCurrentX;
        float cubeCurrentY;
        float cubeCurrentZ;
//        Matrix4x4 controllerMatrix;
    };
/**
 * This is a sample app for the Cardboard SDK. It loads a simple environment and
 * objects that you can click on.
 */
    class HelloCardboardApp {
    public:
        /**
         * Creates a HelloCardboardApp.
         *
         * @param vm JavaVM pointer.
         * @param obj Android activity object.
         * @param asset_mgr_obj The asset manager object.
         */
        HelloCardboardApp(JavaVM* vm, jobject obj, jobject asset_mgr_obj);

        ~HelloCardboardApp();

        /**
         * Initializes any GL-related objects. This should be called on the rendering
         * thread with a valid GL context.
         *
         * @param env The JNI environment.
         */
        void OnSurfaceCreated(JNIEnv* env);

        /**
         * Sets screen parameters.
         *
         * @param width Screen width
         * @param height Screen height
         */
        void SetScreenParams(int width, int height);

        /**
         * Draws the scene. This should be called on the rendering thread.
         */
        void OnDrawFrame();

        /**
         * Hides the target object if it's being targeted.
         */
        void OnTriggerEvent(jfloat d, jfloat d1);

        /**
         * Pauses head tracking.
         */
        void OnPause();

        /**
         * Resumes head tracking.
         */
        void OnResume();

        /**
         * Allows user to switch viewer.
         */
        void SwitchViewer();

    private:
        /**
         * Default near clip plane z-axis coordinate.
         */
        static constexpr float kZNear = 0.1f;

        /**
         * Default far clip plane z-axis coordinate.
         */
        static constexpr float kZFar = 100.f;

        /**
         * Updates device parameters, if necessary.
         *
         * @return true if device parameters were successfully updated.
         */
        bool UpdateDeviceParams();

        /**
         * Initializes GL environment.
         */
        void GlSetup();

        /**
         * Deletes GL environment.
         */
        void GlTeardown();

        /**
         * Gets head's pose as a 4x4 matrix.
         *
         * @return matrix containing head's pose.
         */
        Matrix4x4 GetPose();

        /**
         * Draws the target object.
         */
        void DrawTarget();

        /**
         * Draws the room.
         */
        void DrawRoom();

        /**
         * Draws the cube
         */
        void DrawCube();
        void DrawCube(Matrix4x4 projection_matrix_);
        void DrawCube(Matrix4x4 projection_matrix_, int texture_id);

//        /**
//         * Draws the controller
//         */
//        void DrawController();
//        void DrawController(Matrix4x4 projection_matrix_);
//        void DrawController(Matrix4x4 projection_matrix_, int texture_id);

        /**
         * Finds a new random position for the target object.
         */
        void HideTarget();

        /**
         * Checks if user is pointing or looking at the target object by calculating
         * whether the angle between the user's gaze and the vector pointing towards
         * the object is lower than some threshold.
         *
         * @return true if the user is pointing at the target object.
         */
        bool IsPointingAtTarget();
        bool IsPointingAtTarget(Matrix4x4 target_position_matrix_param);
//        float cubeCurrentX;
//        float cubeCurrentY;
//        float cubeCurrentZ;
        float cubeVx;
        float cubeVy;
        float cubeVz;
        float roomWidth;
        float roomHeight;

        bool isHeadset;
        bool isController;

        jobject java_asset_mgr_;
        AAssetManager* asset_mgr_;

        CardboardHeadTracker* head_tracker_;
        CardboardLensDistortion* lens_distortion_;
        CardboardDistortionRenderer* distortion_renderer_;

        CardboardEyeTextureDescription left_eye_texture_description_;
        CardboardEyeTextureDescription right_eye_texture_description_;

        bool screen_params_changed_;
        bool device_params_changed_;
        int screen_width_;
        int screen_height_;

        float projection_matrices_[2][16];
        float eye_matrices_[2][16];

        GLuint depthRenderBuffer_;  // depth buffer
        GLuint framebuffer_;        // framebuffer object
        GLuint texture_;            // distortion texture

        GLuint obj_program_;
        GLuint obj_position_param_;
        GLuint obj_uv_param_;
        GLuint obj_modelview_projection_param_;

        Matrix4x4 head_view_;
        Matrix4x4 target_position_matrix;
        Matrix4x4 cube_position_matrix;

        Matrix4x4 target_projection_matrix_;
        Matrix4x4 roomProjectionMatrix_;
        Matrix4x4 cube_projection_matrix_;


        TexturedMesh room_;
        TexturedMesh cube_;
//        TexturedMesh controller_;
        Texture room_tex_;
        Texture cube_tex_;
        Texture cube_tex_selected_;
//        Texture controller_tex_;

        std::vector<TexturedMesh> target_object_meshes_;
        std::vector<Texture> target_object_not_selected_textures_;
        std::vector<Texture> target_object_selected_textures_;
        int cur_target_object_;
        std::vector<Texture> texture_array_;
        std::vector<Matrix4x4> initMenuCoordinatesArray_; // All the cubes that are displayed in the init menu screen

        // Initial position of the player
        float viewer_position_x;
        float viewer_position_y;
        float viewer_position_z;

        bool isCubeMoving;
        void toggleCubeMovement();
        float random(float a, float b);

        void computeRandomSpeed();

        void boundSpeed();

        void OnTriggerEventHeadset();

        void OnTriggerEventController(jfloat d, jfloat d1);

        void DefineHeadsetOrController();

        void DrawWorld(Matrix4x4 projectionMatrix);

        void DrawInitMenu(Matrix4x4 projectionMatrix);

        int nbMenuRows = 7;
        int nbMenuCubesPerRow = 13;
        void setupInitMenuCoordinates();

        int getInitMenuValue(int cube_index);

        void setupServer();

        void OnDrawFrameHeadset();

        void OnDrawFrameController();

        void closeSockets();

        void setupServerForController() const;

        void setupServerForHeadset() const;

        void sendMessageToHeadset(char *message) const;

        public: static bool startswith(std::string s, std::string sub);

        static vector<std::string> string_split(std::string &message, const std::string &delimiter);
    };

}  // namespace ndk_hello_cardboard

#endif  // HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_
