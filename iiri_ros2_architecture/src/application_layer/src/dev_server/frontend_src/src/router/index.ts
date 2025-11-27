import { createRouter, createWebHistory } from 'vue-router'

const router = createRouter({
    history: createWebHistory(import.meta.env.BASE_URL),
    routes: [
        {
            path: '/',
            name: 'home',
            component: () => import('../views/HomeView.vue')
        },
        {
            path: '/ota',
            name: 'ota-update',
            component: () => import('../views/OTAUpdate.vue')
        },
        {
            path: '/version',
            name: 'version',
            component: () => import('../views/VersionView.vue')
        }
    ]
})

export default router