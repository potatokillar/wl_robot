/*
 * @Author: 唐文浩
 * @Date: 2024-11-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-11-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
// #include "nlsToken.h"
#ifndef NLS_TOKEN_C_API_H_
#define NLS_TOKEN_C_API_H_

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void *NlsTokenHandle;
    // typedef void *NlsClientHandle;

    NlsTokenHandle nls_token_create(); // 创建 NlsToken 对象
    // NlsClientHandle nls_client_create();  // 创建 NlsClient 对象
    void nls_token_destroy(NlsTokenHandle handle); // 销毁 NlsToken 对象
    // 调用 NlsToken 的成员函数
    void nls_token_setAccessKeyId(NlsTokenHandle handle, const char *accessKeyId);
    void nls_token_setKeySecret(NlsTokenHandle handle, const char *KeySecret);
    void nls_token_setDomain(NlsTokenHandle handle, const char *domain);
    void nls_token_setServerVersion(NlsTokenHandle handle, const char *serverVersion);
    // void nls_client_vipServerListGetUrl(NlsTokenHandle handle, const char *vipServerDomainList, const char *targetDomain, char *url);

#ifdef __cplusplus
}
#endif

#endif // NLS_TOKEN_C_API_H_