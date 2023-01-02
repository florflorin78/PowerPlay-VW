/*
 * Copyright (C) 2020 The Android Open Source Project
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

package com.android.systemui.statusbar.notification.row.dagger;

import com.android.systemui.statusbar.notification.row.ActivatableNotificationView;
import com.android.systemui.statusbar.notification.row.ActivatableNotificationViewController;

import dagger.BindsInstance;
import dagger.Subcomponent;

/**
 * Dagger subcomponent for Notification related views.
 */
@Subcomponent(modules = {ActivatableNotificationViewModule.class})
@NotificationRowScope
public interface NotificationRowComponent {
    /**
     * Builder for {@link NotificationRowComponent}.
     */
    @Subcomponent.Builder
    interface Builder {
        @BindsInstance
        Builder activatableNotificationView(ActivatableNotificationView view);
        NotificationRowComponent build();
    }

    /**
     * Creates a ActivatableNotificationViewController.
     */
    @NotificationRowScope
    ActivatableNotificationViewController getActivatableNotificationViewController();
}
