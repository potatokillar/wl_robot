/*
 * @Author: 唐文浩
 * @Date: 2024-11-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-11-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
// 自己编写的头文件
#include "nlsToken_c_api.h"
// 阿里云原有的头文件
#include "nlsClient.h"
#include "nlsToken.h" // 原始 C++ 动态库的头文件

NlsTokenHandle nls_token_create() { return reinterpret_cast<NlsTokenHandle>(new AlibabaNlsCommon::NlsToken()); }

// NlsClientHandle nls_client_create(){ return reinterpret_cast<NlsTokenHandle>(new AlibabaNlsCommon::NlsToken()); }

void nls_token_destroy(NlsTokenHandle handle) { delete reinterpret_cast<AlibabaNlsCommon::NlsToken *>(handle); }

void nls_token_setAccessKeyId(NlsTokenHandle handle, const char *accessKeyId)
{
    AlibabaNlsCommon::NlsToken *token = reinterpret_cast<AlibabaNlsCommon::NlsToken *>(handle);
    token->setAccessKeyId(accessKeyId);
}

void nls_token_setKeySecret(NlsTokenHandle handle, const char *KeySecret)
{
    AlibabaNlsCommon::NlsToken *token = reinterpret_cast<AlibabaNlsCommon::NlsToken *>(handle);
    token->setKeySecret(KeySecret);
}

void nls_token_setDomain(NlsTokenHandle handle, const char *domain)
{
    AlibabaNlsCommon::NlsToken *token = reinterpret_cast<AlibabaNlsCommon::NlsToken *>(handle);
    token->setKeySecret(domain);
}

void nls_token_setServerVersion(NlsTokenHandle handle, const char *serverVersion)
{
    AlibabaNlsCommon::NlsToken *token = reinterpret_cast<AlibabaNlsCommon::NlsToken *>(handle);
    token->setServerVersion(serverVersion);
}
