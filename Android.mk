LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
$(warning 'solen uart_test')
$(warning 'PRODUCT_OUT =',$(PRODUCT_OUT))
LOCAL_SRC_FILES:=uart_test.c
LOCAL_MODULE := uart_test
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/bin
LOCAL_MODULE_TAGS := optional
LOCAL_SHARED_LIBRARIES:= libcutils libutils
include $(BUILD_EXECUTABLE)
